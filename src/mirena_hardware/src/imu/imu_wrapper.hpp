#pragma once

#include <rclcpp/rclcpp.hpp>

#include <vectornav/Interface/Sensor.hpp>
#include <vectornav/Interface/Registers.hpp>

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdint>
#include <mutex>
#include <optional>
#include <string>
#include <vector>

namespace mirena::IMU
{
    /// VN-300 base IMU rate. Fixed by the hardware, not a choice. Lives here
    /// because both the divisor the node computes and the link budget the
    /// wrapper checks are derived from it.
    constexpr int kBaseRateHz = 400;

    /// Everything this driver configures on the VN-300.
    ///
    /// The antenna geometry is per-car and the filter is wrong without it. The
    /// rest describes how the onboard INS is set up, and used to be left at
    /// whatever the unit happened to have saved -- which meant a sensor that
    /// had been bench-tested in AHRS mode would silently run the whole event
    /// without GNSS heading.
    struct Config
    {
        std::vector<double> antenna_a_offset{-0.11, 0.0, 0.52};
        std::vector<double> gnss_baseline{1.50, 0.0, 0.0};
        std::vector<double> gnss_baseline_unc{0.02, 0.02, 0.02};
        uint16_t rate_divisor = 4;

        /// Register 67. 3 = DualGnssNoPressure: dual antenna, no barometric
        /// aiding. Pressure aiding is left off deliberately -- the aero on this
        /// car moves static pressure around the sensor far more than the
        /// altitude signal it would contribute is worth.
        uint8_t ins_scenario = 3;
        bool ahrs_aiding = true;
        /// Let the unit refine the compass baseline it was given. Off by
        /// default because the estimate only converges after real driving; the
        /// estimate_gnss_baseline service turns it on deliberately.
        bool est_baseline = false;

        /// Register 144. Off means the INS ignores the second antenna and
        /// heading becomes observable only under acceleration.
        bool use_gnss_compass = true;

        /// Register 58. The receiver supports 1 Hz and 5 Hz only; anything >= 5
        /// selects 5 Hz. This is the rate the fix actually updates at, whatever
        /// rate the binary message streams at.
        int gnss_rate_hz = 5;

        /// Save to flash once the configuration is correct, so a power cycle
        /// boots already configured and this driver never has to restart the
        /// alignment. Only ever written when something actually changed --
        /// flash has a finite write budget.
        bool persist_config = true;
    };

    /// Register 97: the baseline the unit worked out for itself.
    struct EstimatedBaseline
    {
        bool complete = false;
        uint16_t num_meas = 0;
        double x = 0, y = 0, z = 0;
        double ux = 0, uy = 0, uz = 0;
    };

    /// Registers 86 and 98: why the compass is or is not converging.
    struct CompassHealth
    {
        float sats_a = 0, sats_b = 0;
        float cn0_a = 0, cn0_b = 0;
        float common_sats = 0;
        uint8_t startup_percent = 0;
        float startup_heading = 0;
    };

    class Wrapper
    {
        rclcpp::Logger logger;

        /// Guards every command sent down the port.
        ///
        /// The measurement path deliberately does not take it: getNextMeasurement
        /// pops an internal queue that the SDK's own listening thread fills, so
        /// it never touches the command channel. Making it share this mutex
        /// would stall the entire 100 Hz stream behind a baseline estimation
        /// that polls for minutes.
        std::mutex cmd_mutex_;

        std::atomic<bool> imu_is_connected_{false};
        VN::Sensor imu_;

        /// Whether this connect actually altered the unit, and so whether the
        /// flash write at the end has anything to save.
        bool config_changed_ = false;

    public:
        Wrapper(rclcpp::Logger log) : logger(log) {}

        auto get_next_measurement()
        {
            return imu_.getNextMeasurement();
        }

        /// Connect and configure, or leave the port closed. The caller decides
        /// what to do about failure by asking is_connected(); nothing has ever
        /// wanted the specific VN::Error, so it is not returned.
        void connect_and_config(const char *imu_dev_path, const Config &cfg)
        {
            std::lock_guard<std::mutex> lock(cmd_mutex_);

            RCLCPP_INFO(logger, "Connecting to IMU on %s ...", imu_dev_path);
            if (imu_.autoConnect(imu_dev_path) != VN::Error::None)
            {
                RCLCPP_ERROR(logger, "Could not open %s and/or no IMU found on it.", imu_dev_path);
                return;
            }

            config_changed_ = false;

            // --- Antenna A offset (register 57) ---
            VN::Registers::GNSS::GnssAOffset antenna_a;
            antenna_a.positionX = static_cast<float>(cfg.antenna_a_offset[0]);
            antenna_a.positionY = static_cast<float>(cfg.antenna_a_offset[1]);
            antenna_a.positionZ = static_cast<float>(cfg.antenna_a_offset[2]);
            if (!ensure(antenna_a, "GNSS antenna A offset", same_geometry))
            {
                return;
            }

            // --- GNSS compass baseline, antenna B relative to A (register 93) ---
            VN::Registers::GNSSCompass::GnssCompassBaseline baseline;
            baseline.positionX = static_cast<float>(cfg.gnss_baseline[0]);
            baseline.positionY = static_cast<float>(cfg.gnss_baseline[1]);
            baseline.positionZ = static_cast<float>(cfg.gnss_baseline[2]);
            baseline.uncertaintyX = static_cast<float>(cfg.gnss_baseline_unc[0]);
            baseline.uncertaintyY = static_cast<float>(cfg.gnss_baseline_unc[1]);
            baseline.uncertaintyZ = static_cast<float>(cfg.gnss_baseline_unc[2]);
            if (!ensure(baseline, "GNSS compass baseline", same_baseline))
            {
                return;
            }

            // --- GNSS receiver rate (register 58) ---
            if (!apply_gnss_rate(cfg))
            {
                return;
            }

            // --- INS filter setup (register 67) and compass selection (144) ---
            //
            // Read-modify-write rather than write-from-scratch: both registers
            // carry reserved and multi-receiver fields this driver has no
            // opinion about, and inventing values for them is how you end up
            // disabling a GNSS receiver by accident. If the read fails the
            // fields are left alone entirely.
            if (!apply_ins_config(cfg) || !apply_gnss_select(cfg))
            {
                return;
            }

            // --- Binary output 1: exactly the fields this driver consumes, at
            //     the configured rate. Requesting them explicitly guarantees the
            //     optionals read downstream are always present. ---
            VN::Registers::System::BinaryOutput1 bin_out_1;
            bin_out_1.asyncMode.emplace();
            bin_out_1.asyncMode->serial1 = true;
            bin_out_1.asyncMode->serial2 = false;
            bin_out_1.rateDivisor = cfg.rate_divisor;
            // Three timing fields, for three different jobs. timeStartup is
            // the sensor's own monotonic clock (MCU TCXO, +-20 ppm), stamped
            // at the sample -- it is what the published stamps are built on,
            // because it is free of every latency between the sample and this
            // process. timeGps is absolute UTC, kept for the host-clock skew
            // diagnostic and the legacy stamp mode. timeStatus says whether
            // the GPS one means anything yet.
            bin_out_1.time.timeStartup = true;
            bin_out_1.time.timeGps = true;
            bin_out_1.time.timeStatus = true;
            bin_out_1.imu.accel = true;
            bin_out_1.imu.angularRate = true;
            bin_out_1.attitude.quaternion = true;
            bin_out_1.ins.insStatus = true;
            bin_out_1.ins.posLla = true;
            bin_out_1.ins.velBody = true;
            bin_out_1.attitude.linBodyAcc = true;
            // Onboard-filter uncertainties (1 sigma) — used to fill the INS
            // odometry covariance published for the state estimator.
            bin_out_1.ins.posU = true;
            bin_out_1.ins.velU = true;
            bin_out_1.attitude.yprU = true;
            // The GNSS receiver's own solution, which is what /sensors/gps/fix
            // is supposed to be. The INS position was standing in for it, and
            // that is a different quantity in three ways: it is at the IMU
            // rather than at antenna A, it does not exist until the INS
            // converges, and it carries one isotropic sigma where the receiver
            // reports a real per-axis one. Fix type and satellite count come
            // along because a NavSatFix without them cannot say whether it is
            // trustworthy.
            bin_out_1.gnss.gnss1PosLla = true;
            bin_out_1.gnss.gnss1PosUncertainty = true;
            bin_out_1.gnss.gnss1Fix = true;
            bin_out_1.gnss.gnss1NumSats = true;
            // The receiver solves at 1 or 5 Hz while this message streams at
            // 100, so the same solution arrives up to 20 times. GPS time of
            // week is how the pipeline tells a new fix from a repeat -- without
            // it the driver would publish 20 identical NavSatFix messages per
            // solution, and any consumer treating them as independent would
            // shrink its covariance by sqrt(20) on evidence it never received.
            bin_out_1.gnss.gps1Tow = true;

            if (!ensure(bin_out_1, "binary output 1", same_binary_output))
            {
                return;
            }

            warn_if_link_is_tight(cfg);

            // Silence the other two async outputs, in case a previous session
            // or a saved configuration left them streaming into the same port.
            // A default-constructed AsyncMode is serial1 = serial2 = false;
            // rateDivisor is then irrelevant but must still be set, because
            // BinaryOutput::toString() emits nothing at all without it.
            VN::Registers::System::BinaryOutput2 bin_out_2;
            bin_out_2.asyncMode.emplace();
            bin_out_2.rateDivisor = cfg.rate_divisor;
            VN::Registers::System::BinaryOutput3 bin_out_3;
            bin_out_3.asyncMode.emplace();
            bin_out_3.rateDivisor = cfg.rate_divisor;
            if (!ensure(bin_out_2, "binary output 2", same_binary_output) ||
                !ensure(bin_out_3, "binary output 3", same_binary_output))
            {
                return;
            }

            persist_if_changed(cfg);

            imu_is_connected_ = true;
        }

        void disconnect()
        {
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            imu_.disconnect();
            // The flag is the whole point of this call. Leaving it set meant
            // is_connected() answered yes for ever after the first connect, so
            // the reconnect branch in the publish loop was unreachable and one
            // dropped cable wedged the driver until it was restarted.
            imu_is_connected_ = false;
        }

        bool is_connected() const
        {
            return imu_is_connected_;
        }

        /// Turn the unit's own baseline estimator on or off (register 67).
        bool set_est_baseline(bool enable)
        {
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            VN::Registers::INS::InsBasicConfig reg;
            if (imu_.readRegister(&reg) != VN::Error::None)
            {
                return false;
            }
            const auto wanted = enable ? VN::Registers::INS::InsBasicConfig::EstBaseline::Enable
                                       : VN::Registers::INS::InsBasicConfig::EstBaseline::Disable;
            if (reg.estBaseline == wanted)
            {
                return true;
            }
            reg.estBaseline = wanted;
            return imu_.writeRegister(&reg) == VN::Error::None;
        }

        /// Read the unit's estimated baseline (register 97).
        ///
        /// Locks only for the round trip, so a service polling this for minutes
        /// never blocks a disconnect coming from the publish loop.
        std::optional<EstimatedBaseline> read_est_baseline()
        {
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            VN::Registers::GNSSCompass::GnssCompassEstBaseline reg;
            if (imu_.readRegister(&reg) != VN::Error::None)
            {
                return std::nullopt;
            }
            EstimatedBaseline out;
            out.complete = reg.estBaselineComplete != 0;
            out.num_meas = reg.numMeas;
            out.x = reg.positionX;
            out.y = reg.positionY;
            out.z = reg.positionZ;
            out.ux = reg.uncertaintyX;
            out.uy = reg.uncertaintyY;
            out.uz = reg.uncertaintyZ;
            return out;
        }

        /// Read compass signal health (86) and startup progress (98).
        ///
        /// This is what tells a stuck alignment (0 %, one antenna seeing no
        /// satellites) apart from a slow one (climbing steadily). Without it
        /// the only symptom either produces is silence.
        std::optional<CompassHealth> read_compass_health()
        {
            std::lock_guard<std::mutex> lock(cmd_mutex_);
            VN::Registers::GNSSCompass::GnssCompassSignalHealthStatus health;
            VN::Registers::GNSSCompass::GnssCompassStartupStatus startup;
            if (imu_.readRegister(&health) != VN::Error::None ||
                imu_.readRegister(&startup) != VN::Error::None)
            {
                return std::nullopt;
            }
            CompassHealth out;
            out.sats_a = health.numSatsPvtA;
            out.sats_b = health.numSatsPvtB;
            out.cn0_a = health.highestCn0A;
            out.cn0_b = health.highestCn0B;
            out.common_sats = health.numComSatsPvt;
            out.startup_percent = startup.percentComplete;
            out.startup_heading = startup.currentHeading;
            return out;
        }

    private:
        /// Metres. Two orders of magnitude below anything that affects the
        /// solution, and far enough above the ASCII round trip that a register
        /// never reads back as different from what was just written to it.
        static constexpr float kGeometryTol = 1e-4f;

        static bool near(const std::optional<float> &a, const std::optional<float> &b)
        {
            return a.has_value() && b.has_value() &&
                   std::fabs(a.value() - b.value()) <= kGeometryTol;
        }

        static bool same_geometry(const VN::Registers::GNSS::GnssAOffset &a,
                                  const VN::Registers::GNSS::GnssAOffset &b)
        {
            return near(a.positionX, b.positionX) && near(a.positionY, b.positionY) &&
                   near(a.positionZ, b.positionZ);
        }

        static bool same_baseline(const VN::Registers::GNSSCompass::GnssCompassBaseline &a,
                                  const VN::Registers::GNSSCompass::GnssCompassBaseline &b)
        {
            return near(a.positionX, b.positionX) && near(a.positionY, b.positionY) &&
                   near(a.positionZ, b.positionZ) && near(a.uncertaintyX, b.uncertaintyX) &&
                   near(a.uncertaintyY, b.uncertaintyY) && near(a.uncertaintyZ, b.uncertaintyZ);
        }

        /// Binary output is all bitfields and integers, so the SDK's own
        /// comparison is exact and correct here.
        static bool same_binary_output(const VN::Registers::System::BinaryOutput &a,
                                       const VN::Registers::System::BinaryOutput &b)
        {
            return a == b;
        }

        /// Write `desired` only if the unit does not already hold it.
        ///
        /// This is the reason this class exists. Writing the antenna geometry
        /// registers restarts the GNSS compass alignment, so the previous
        /// version -- which wrote every register unconditionally on every
        /// connect -- threw away convergence it had already paid for, once per
        /// launch and again on every reconnect after a read timeout. On a car
        /// that is staged for minutes and then runs for seconds, that is the
        /// difference between arriving at the line converged and arriving
        /// aligning. One extra round trip buys it back.
        ///
        /// A failed read falls through to writing: the safe direction is the
        /// old behaviour, not skipping configuration on a guess.
        template <typename Reg, typename Eq>
        bool ensure(Reg &desired, const char *what, Eq equal)
        {
            Reg current;
            if (imu_.readRegister(&current) == VN::Error::None && equal(current, desired))
            {
                RCLCPP_INFO(logger, "%s already matches; leaving it (no alignment restart).", what);
                return true;
            }

            if (imu_.writeRegister(&desired) != VN::Error::None)
            {
                RCLCPP_WARN(logger, "Could not write %s register; closing the port to retry.", what);
                imu_.disconnect();
                return false;
            }
            RCLCPP_INFO(logger, "%s updated.", what);
            config_changed_ = true;
            return true;
        }

        bool apply_ins_config(const Config &cfg)
        {
            using Reg = VN::Registers::INS::InsBasicConfig;
            Reg reg;
            if (imu_.readRegister(&reg) != VN::Error::None)
            {
                RCLCPP_WARN(logger, "Could not read INS basic config (register 67); "
                                    "leaving the INS scenario as the unit has it.");
                return true;
            }

            const auto scenario = static_cast<Reg::Scenario>(cfg.ins_scenario);
            const auto ahrs = cfg.ahrs_aiding ? Reg::AhrsAiding::Enable : Reg::AhrsAiding::Disable;
            const auto est = cfg.est_baseline ? Reg::EstBaseline::Enable : Reg::EstBaseline::Disable;

            if (reg.scenario == scenario && reg.ahrsAiding == ahrs && reg.estBaseline == est)
            {
                RCLCPP_INFO(logger, "INS basic config already matches (scenario %u); leaving it.",
                            static_cast<unsigned>(cfg.ins_scenario));
                return true;
            }

            reg.scenario = scenario;
            reg.ahrsAiding = ahrs;
            reg.estBaseline = est;
            if (imu_.writeRegister(&reg) != VN::Error::None)
            {
                RCLCPP_WARN(logger, "Could not write INS basic config; closing the port to retry.");
                imu_.disconnect();
                return false;
            }
            RCLCPP_INFO(logger, "INS basic config updated: scenario %u, AHRS aiding %s, baseline estimation %s.",
                        static_cast<unsigned>(cfg.ins_scenario),
                        cfg.ahrs_aiding ? "on" : "off", cfg.est_baseline ? "on" : "off");
            config_changed_ = true;
            return true;
        }

        /// Register 58. The receiver supports 1 Hz and 5 Hz and nothing else.
        ///
        /// Worth setting explicitly rather than inheriting: at 1 Hz the INS gets
        /// a fifth of the position and compass updates, which shows up as
        /// slower convergence and a looser solution, and nothing in the stack
        /// would say why.
        bool apply_gnss_rate(const Config &cfg)
        {
            using Reg = VN::Registers::GNSS::GnssBasicConfig;
            Reg reg;
            if (imu_.readRegister(&reg) != VN::Error::None)
            {
                RCLCPP_WARN(logger, "Could not read GNSS basic config (register 58); "
                                    "leaving the receiver rate as the unit has it.");
                return true;
            }

            const auto wanted = cfg.gnss_rate_hz >= 5 ? Reg::Rate::Rate5Hz : Reg::Rate::Rate1Hz;
            if (reg.rate == wanted)
            {
                return true;
            }
            reg.rate = wanted;
            if (imu_.writeRegister(&reg) != VN::Error::None)
            {
                RCLCPP_WARN(logger, "Could not write GNSS basic config; closing the port to retry.");
                imu_.disconnect();
                return false;
            }
            RCLCPP_INFO(logger, "GNSS receiver rate set to %d Hz.",
                        wanted == Reg::Rate::Rate5Hz ? 5 : 1);
            config_changed_ = true;
            return true;
        }

        bool apply_gnss_select(const Config &cfg)
        {
            using Reg = VN::Registers::INS::InsGnssSelect;
            Reg reg;
            if (imu_.readRegister(&reg) != VN::Error::None)
            {
                RCLCPP_WARN(logger, "Could not read INS GNSS select (register 144); "
                                    "leaving GNSS compass selection as the unit has it.");
                return true;
            }

            const auto wanted = cfg.use_gnss_compass ? Reg::UseGnssCompass::On : Reg::UseGnssCompass::Off;
            if (reg.useGnssCompass == wanted)
            {
                return true;
            }
            reg.useGnssCompass = wanted;
            if (imu_.writeRegister(&reg) != VN::Error::None)
            {
                RCLCPP_WARN(logger, "Could not write INS GNSS select; closing the port to retry.");
                imu_.disconnect();
                return false;
            }
            RCLCPP_INFO(logger, "GNSS compass %s.", cfg.use_gnss_compass ? "enabled" : "disabled");
            config_changed_ = true;
            return true;
        }

        /// Check the serial link can actually carry what was just requested.
        ///
        /// Worth doing because the binary message is assembled field by field
        /// well away from any thought about bandwidth, and the failure mode of
        /// overrunning the link is not an error but dropped and truncated
        /// messages -- which surface downstream as an IMU that mysteriously
        /// streams below its configured rate.
        void warn_if_link_is_tight(const Config &cfg)
        {
            const auto baud = imu_.connectedBaudRate();
            if (!baud.has_value())
            {
                return;
            }
            const double rate_hz = static_cast<double>(kBaseRateHz) / std::max<uint16_t>(1, cfg.rate_divisor);
            // Payload of the fields enabled above, plus header and CRC. Counted
            // rather than measured, so it is an estimate -- but the margin the
            // check asks for is generous enough to absorb the error.
            constexpr double kMessageBytes = 183.0;
            const double needed_bps = kMessageBytes * rate_hz * 10.0; // 8N1 framing
            const double available_bps = static_cast<double>(*baud);

            RCLCPP_INFO(logger, "Serial link at %.0f baud; binary output 1 needs about %.0f bps at %.0f Hz.",
                        available_bps, needed_bps, rate_hz);

            if (needed_bps > 0.8 * available_bps)
            {
                RCLCPP_WARN(logger,
                            "The requested binary output needs ~%.0f bps but the link runs at %.0f baud. "
                            "Expect dropped or truncated measurements. Raise the unit's baud rate or "
                            "lower imu_output_rate_hz.",
                            needed_bps, available_bps);
            }
        }

        /// Save to flash, but only when this connect actually changed something.
        ///
        /// Unconditional saving would burn a finite flash write budget on every
        /// launch for no benefit; the point of saving at all is that the *next*
        /// power cycle finds the unit already correct and skips straight past
        /// every write above.
        void persist_if_changed(const Config &cfg)
        {
            if (!cfg.persist_config)
            {
                return;
            }
            if (!config_changed_)
            {
                RCLCPP_INFO(logger, "Configuration already saved on the unit; nothing to persist.");
                return;
            }
            if (imu_.writeSettings() != VN::Error::None)
            {
                RCLCPP_WARN(logger, "Could not save configuration to flash. The unit is configured "
                                    "for this session, but will need reconfiguring after a power cycle.");
                return;
            }
            RCLCPP_INFO(logger, "Configuration saved to flash; a power cycle will boot configured.");
        }
    };
}
