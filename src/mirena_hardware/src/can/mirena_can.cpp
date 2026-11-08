// SocketCAN bridge node for the TER DBC (hardware).
//
// Implements five paths:
//   * CarControl (ROS) -> DV_Dynamic_Req_1 (CAN)        [output]
//   * MissionInfo (ROS) -> DV_info (CAN)                [output]
//   * DV_system_status (CAN) -> AsInfo (ROS)            [input]
//   * WheelInfo (CAN) -> WheelSpeeds (ROS)              [input]
//   * APPS/BPPS/STEER (CAN) -> CarControl (ROS)         [input]
//
// The last path mirrors the manual (pedal) driving commands, so a manual test can
// be recorded as a ROS bag: every APPS, BPPS or STEER frame received republishes a
// CarControl carrying the latest known gas and steering values. It goes to its own
// topic, never /control -- that topic is this node's CAN TX source, and feeding it
// back would close a loop through the car.
//
// ...and, optionally, records the whole bus to disk. See can_logger.hpp for how
// that is kept off this node's rx path; the policy of when to record lives here,
// because it is a fact about the TER state machine rather than about logging:
// a session runs from TER_STATUS reaching PRECHARGED until it returns to
// WAIT_SL, which is exactly the window in which the HV system is live.
//
// The CAN interface is expected to be already up (bitrate configured, link up);
// bringing it up is the responsibility of the system (e.g. systemd-networkd / ip link).
// CAN (de)serialization is delegated to the cantools-generated C library (ter.h/ter.c).

#include <cstdint>
#include <cstring>
#include <string>
#include <memory>
#include <chrono>
#include <optional>
#include <algorithm>

#include <unistd.h>
#include <fcntl.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <linux/can.h>
#include <linux/can/raw.h>

#include "can_logger.hpp"

#include "rclcpp/rclcpp.hpp"

#include "mirena_common/msg/car_control.hpp"
#include "mirena_common/msg/as_info.hpp"
#include "mirena_common/msg/mission_info.hpp"
#include "mirena_common/msg/wheel_speeds.hpp"

extern "C" {
#include "ter.h"
}

using namespace std::chrono_literals;
using std::placeholders::_1;

namespace
{
// Wheel speeds are a body-frame quantity, so they are stamped against the frame
// every other body-frame quantity in this stack uses (Car.u/v, the IMU).
constexpr const char * kWheelFrame = "car/cog";
}  // namespace

class MirenaCan : public rclcpp::Node
{
public:
  MirenaCan()
  : rclcpp::Node("mirena_can")
  {
    iface_ = declare_parameter<std::string>("can_interface", "can1");

    // BPPS full scale in Bar, used to normalize brake pressure into [0,1] of gas.
    // 1.0 Bar is what the DBC declares; raise it if the real pedal reads higher.
    bpps_full_scale_bar_ = declare_parameter<double>("bpps_full_scale_bar", 1.0);

    // Recording is off unless asked for, and asking for it without saying where
    // is a configuration mistake rather than a reason to pick a directory on
    // the user's behalf -- a log written somewhere nobody looks is worse than
    // no log, because it is believed to exist.
    const bool log_enabled = declare_parameter<bool>("log", true);
    const std::string log_path = declare_parameter<std::string>("log_path", "/home/mirena/mirena_ws/src/mirena_hardware/src/can/log");
    if (log_enabled && log_path.empty()) {
      RCLCPP_ERROR(get_logger(), "log is set but log_path is empty; not recording.");
    } else if (log_enabled) {
      logger_ = std::make_unique<mirena::CanLogger>(log_path, iface_);
      RCLCPP_INFO(
        get_logger(),
        "Recording to '%s'. A session opens when TER_STATUS reaches PRECHARGED "
        "and closes when it returns to WAIT_SL.", log_path.c_str());
    }

    if (!open_can(iface_)) {
      // Keep the node alive so the rest of the stack still launches; CAN I/O is gated on the fd.
      RCLCPP_ERROR(get_logger(), "CAN bus '%s' unavailable; node running but inactive", iface_.c_str());
    } else {
      RCLCPP_INFO(get_logger(), "CAN bus opened on interface '%s'", iface_.c_str());
    }

    control_sub_ = create_subscription<mirena_common::msg::CarControl>(
      "/control", 10, std::bind(&MirenaCan::control_callback, this, _1));

    mission_info_sub_ = create_subscription<mirena_common::msg::MissionInfo>(
      "/system/mission_status", 10, std::bind(&MirenaCan::mission_info_callback, this, _1));

    as_info_pub_ = create_publisher<mirena_common::msg::AsInfo>("/as_status", 10);

    // Sensor data, so it follows the same best-effort contract as the other
    // /sensors topics: a wheel speed that arrived late describes a speed the car
    // is no longer doing, and is better dropped than queued.
    wheel_speeds_pub_ = this->create_publisher<mirena_common::msg::WheelSpeeds>(
    "/sensors/wheel_speeds", rclcpp::SensorDataQoS());

    // The same contract as /sensors: these are commands sampled from pedals, so a
    // message that arrives late describes an input the driver has already released.
    // Dropping it is better than queueing it behind the ones after it.
    const std::string car_control_topic = declare_parameter<std::string>("car_control_topic", "/control/can");
    car_control_pub_ = create_publisher<mirena_common::msg::CarControl>(car_control_topic, rclcpp::SensorDataQoS());

    // Poll the socket regularly; reads are non-blocking and drain all pending frames.
    rx_timer_ = create_wall_timer(5ms, std::bind(&MirenaCan::rx_poll, this));
  }

  ~MirenaCan() override
  {
    // A run cut short by Ctrl-C or a supervisor teardown never sees WAIT_SL, so
    // the session is closed here too rather than left to the logger's own
    // destructor -- same flush, but it says so in the log.
    if (logger_) {
      close_session();
    }
    close_can();
  }

private:
  // --- CAN socket ---
  bool open_can(const std::string & iface)
  {
    can_fd_ = socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (can_fd_ < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to create CAN socket: %s", std::strerror(errno));
      return false;
    }

    struct ifreq ifr {};
    std::strncpy(ifr.ifr_name, iface.c_str(), IFNAMSIZ - 1);
    if (ioctl(can_fd_, SIOCGIFINDEX, &ifr) < 0) {
      RCLCPP_ERROR(get_logger(), "Interface '%s' not found: %s", iface.c_str(), std::strerror(errno));
      close_can();
      return false;
    }

    // What the kernel is allowed to hand us depends on whether we are recording.
    //
    // Not recording: only the frames this node decodes. Everything else is
    // discarded in the kernel and never costs us a read.
    //
    // Recording: no filter at all, because "log all CAN frames" cannot be done
    // behind a filter that drops most of them -- and TER_STATUS, which decides
    // when a session runs, is itself one of the frames a narrow filter would
    // throw away. Error frames are enabled too: bus-off, error-passive and
    // arbitration-lost are the events worth having a log for in the first
    // place, and they are invisible without CAN_RAW_ERR_FILTER.
    if (logger_) {
      const can_err_mask_t err_mask = CAN_ERR_MASK;
      if (setsockopt(can_fd_, SOL_CAN_RAW, CAN_RAW_ERR_FILTER, &err_mask, sizeof(err_mask)) < 0) {
        // Not fatal: the log is merely poorer for it.
        RCLCPP_WARN(
          get_logger(), "Could not subscribe to CAN error frames: %s", std::strerror(errno));
      }
    } else {
      // The frames we transmit (DV_Dynamic_Req_1, DV_info) are deliberately absent:
      // nothing we send should come back to us as a reading of the car.
      struct can_filter rfilter[5];
      const canid_t wanted[] = {
        TER_DV_SYSTEM_STATUS_FRAME_ID,
        TER_WHEEL_INFO_FRAME_ID,
        TER_APPS_FRAME_ID,
        TER_BPPS_FRAME_ID,
        TER_STEER_FRAME_ID,
      };
      for (size_t i = 0; i < sizeof(wanted) / sizeof(wanted[0]); ++i) {
        rfilter[i].can_id = wanted[i];
        rfilter[i].can_mask = CAN_SFF_MASK;  // all of these are standard (11-bit) frames
      }
      if (setsockopt(can_fd_, SOL_CAN_RAW, CAN_RAW_FILTER, &rfilter, sizeof(rfilter)) < 0) {
        RCLCPP_ERROR(get_logger(), "Failed to set CAN filter: %s", std::strerror(errno));
        close_can();
        return false;
      }
    }

    // Kernel receive timestamps. Without these every frame drained in one poll
    // would carry the same userspace instant, quantising the log to the 5 ms
    // timer and destroying exactly the timing detail a bus trace is kept for.
    const int on = 1;
    if (setsockopt(can_fd_, SOL_SOCKET, SO_TIMESTAMP, &on, sizeof(on)) < 0) {
      RCLCPP_WARN(
        get_logger(), "SO_TIMESTAMP unavailable (%s); falling back to arrival time.",
        std::strerror(errno));
    }

    struct sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (bind(can_fd_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
      RCLCPP_ERROR(get_logger(), "Failed to bind CAN socket: %s", std::strerror(errno));
      close_can();
      return false;
    }

    const int flags = fcntl(can_fd_, F_GETFL, 0);
    fcntl(can_fd_, F_SETFL, flags | O_NONBLOCK);
    return true;
  }

  void close_can()
  {
    if (can_fd_ >= 0) {
      ::close(can_fd_);
      can_fd_ = -1;
    }
  }

  // --- ROS CarControl -> CAN DV_Dynamic_Req_1 ---
  void control_callback(const mirena_common::msg::CarControl::SharedPtr msg)
  {
    if (can_fd_ < 0) {
      return;
    }

    struct ter_dv_dynamic_req_1_t src;
    ter_dv_dynamic_req_1_init(&src);
    src.trq_req = ter_dv_dynamic_req_1_trq_req_encode(static_cast<float>(msg->gas));
    src.steer_angle_req = ter_dv_dynamic_req_1_steer_angle_req_encode(static_cast<float>(msg->steer_angle));

    struct can_frame frame {};
    frame.can_id = TER_DV_DYNAMIC_REQ_1_FRAME_ID;  // standard frame, no flags
    frame.can_dlc = TER_DV_DYNAMIC_REQ_1_LENGTH;

    const int packed = ter_dv_dynamic_req_1_pack(frame.data, &src, sizeof(frame.data));
    if (packed < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Failed to pack DV_Dynamic_Req_1");
      return;
    }

    if (write(can_fd_, &frame, sizeof(frame)) != static_cast<ssize_t>(sizeof(frame))) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Failed to write DV_Dynamic_Req_1 to CAN: %s", std::strerror(errno));
    }
  }

  // --- ROS MissionInfo -> CAN DV_info ---
  void mission_info_callback(const mirena_common::msg::MissionInfo::SharedPtr msg)
  {
    if (can_fd_ < 0) {
      return;
    }

    struct ter_dv_info_t src;
    ter_dv_info_init(&src);

    src.mission_status = map_mission_status(msg->status);
    // DV_info packs these signals into limited bit-widths (see ter.c pack): the
    // encode helpers only cast, so clamp to the wire range first.
    src.cones_count_actual =
      ter_dv_info_cones_count_actual_encode(static_cast<float>(std::min<uint32_t>(msg->cones_count_actual, 255u)));
    src.cones_count_all =
      ter_dv_info_cones_count_all_encode(static_cast<float>(std::min<uint32_t>(msg->cones_count_total, 131071u)));
    src.lap_counter =
      ter_dv_info_lap_counter_encode(static_cast<float>(std::min<uint32_t>(msg->lap_counter, 15u)));
    // MissionInfo carries no mission type; report MANUAL.
    src.mission = TER_DV_INFO_MISSION_MANUAL_CHOICE;

    struct can_frame frame {};
    frame.can_id = TER_DV_INFO_FRAME_ID;  // standard frame, no flags
    frame.can_dlc = TER_DV_INFO_LENGTH;

    const int packed = ter_dv_info_pack(frame.data, &src, sizeof(frame.data));
    if (packed < 0) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Failed to pack DV_info");
      return;
    }

    if (write(can_fd_, &frame, sizeof(frame)) != static_cast<ssize_t>(sizeof(frame))) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000,
        "Failed to write DV_info to CAN: %s", std::strerror(errno));
    }
  }

  /// Kernel receive time of the frame just read, in nanoseconds since the
  /// epoch, falling back to now if the socket did not supply one.
  static uint64_t receive_stamp_ns(const struct msghdr & msg)
  {
    for (const struct cmsghdr * cmsg = CMSG_FIRSTHDR(&msg); cmsg != nullptr;
      cmsg = CMSG_NXTHDR(const_cast<struct msghdr *>(&msg), const_cast<struct cmsghdr *>(cmsg)))
    {
      if (cmsg->cmsg_level == SOL_SOCKET && cmsg->cmsg_type == SO_TIMESTAMP &&
        cmsg->cmsg_len >= CMSG_LEN(sizeof(struct timeval)))
      {
        struct timeval tv;
        std::memcpy(&tv, CMSG_DATA(cmsg), sizeof(tv));
        return static_cast<uint64_t>(tv.tv_sec) * 1000000000ull +
               static_cast<uint64_t>(tv.tv_usec) * 1000ull;
      }
    }
    // System clock, not the node clock: these stamps are compared against
    // candump output and other machines' logs, so they have to be wall time.
    return static_cast<uint64_t>(
      std::chrono::duration_cast<std::chrono::nanoseconds>(
        std::chrono::system_clock::now().time_since_epoch()).count());
  }

  /// The TER_STATUS state signal, if this frame is a TER_STATUS that unpacks.
  static std::optional<uint8_t> ter_state(const struct can_frame & frame)
  {
    struct ter_ter_status_t dst;
    if (ter_ter_status_unpack(&dst, frame.data, frame.can_dlc) != 0) {
      return std::nullopt;
    }
    return dst.state;
  }

  // --- CAN -> ROS ---
  void rx_poll()
  {
    if (can_fd_ < 0) {
      return;
    }

    struct can_frame frame;
    struct iovec iov;
    struct msghdr msg;
    char control[CMSG_SPACE(sizeof(struct timeval))];

    while (true) {
      iov.iov_base = &frame;
      iov.iov_len = sizeof(frame);
      std::memset(&msg, 0, sizeof(msg));
      msg.msg_iov = &iov;
      msg.msg_iovlen = 1;
      msg.msg_control = control;
      msg.msg_controllen = sizeof(control);

      const ssize_t n = recvmsg(can_fd_, &msg, 0);
      if (n < 0) {
        break;  // EAGAIN / EWOULDBLOCK: no more frames pending
      }
      if (n < static_cast<ssize_t>(sizeof(frame))) {
        continue;
      }

      // Only a standard data frame can be one of ours. This matters now that
      // recording runs the socket unfiltered: CAN_SFF_MASK keeps the low 11
      // bits of an extended id too, so without this test a 29-bit frame ending
      // in 0x27 would be decoded as WheelInfo, and an error frame would be
      // decoded as whatever its flags happened to alias onto.
      const bool standard_data =
        (frame.can_id & (CAN_EFF_FLAG | CAN_RTR_FLAG | CAN_ERR_FLAG)) == 0u;
      const canid_t id = frame.can_id & CAN_SFF_MASK;

      // The session opens before this frame is logged and closes after it, so
      // the PRECHARGED frame that starts a recording and the WAIT_SL frame that
      // ends it are both inside the file rather than straddling its edges.
      std::optional<uint8_t> state;
      if (logger_ && standard_data && id == TER_TER_STATUS_FRAME_ID) {
        state = ter_state(frame);
        if (state == TER_TER_STATUS_STATE_PRECHARGED_CHOICE) {
          open_session();
        }
      }

      if (logger_) {
        logger_->write(receive_stamp_ns(msg), frame.can_id, frame.can_dlc, frame.data);
      }

      if (state == TER_TER_STATUS_STATE_WAIT_SL_CHOICE) {
        close_session();
      }

      if (!standard_data) {
        continue;
      }
      switch (id) {
        case TER_DV_SYSTEM_STATUS_FRAME_ID:
          handle_system_status(frame);
          break;
        case TER_WHEEL_INFO_FRAME_ID:
          handle_wheel_info(frame);
          break;
        case TER_APPS_FRAME_ID:
          handle_apps(frame);
          break;
        case TER_BPPS_FRAME_ID:
          handle_bpps(frame);
          break;
        case TER_STEER_FRAME_ID:
          handle_steer(frame);
          break;
        default:
          break;  // not ours; only reachable while recording runs unfiltered
      }
    }
  }

  // --- Recording sessions, driven by the TER state machine ---
  //
  // Both are idempotent in CanLogger, so they can be called from every
  // TER_STATUS broadcast without tracking edges here; these wrappers exist only
  // to say something in the log the first time each way.

  void open_session()
  {
    if (logger_->active()) {
      return;
    }
    if (logger_->start()) {
      RCLCPP_INFO(get_logger(), "PRECHARGED: recording to '%s'.", logger_->stats().file.c_str());
    } else {
      RCLCPP_ERROR(get_logger(), "PRECHARGED but recording failed: %s",
        logger_->last_error().c_str());
    }
  }

  void close_session()
  {
    if (!logger_->active()) {
      return;
    }
    const auto before = logger_->stats();
    logger_->stop();
    if (before.frames_dropped > 0) {
      RCLCPP_WARN(
        get_logger(),
        "WAIT_SL: closed '%s' with %lu frames, %lu DROPPED -- the writer could not keep up.",
        before.file.c_str(),
        static_cast<unsigned long>(before.frames_written),
        static_cast<unsigned long>(before.frames_dropped));
    } else {
      RCLCPP_INFO(
        get_logger(), "WAIT_SL: closed '%s' with %lu frames.",
        before.file.c_str(), static_cast<unsigned long>(before.frames_written));
    }
  }

  // --- CAN DV_system_status -> ROS AsInfo ---
  void handle_system_status(const struct can_frame & frame)
  {
    struct ter_dv_system_status_t dst;
    if (ter_dv_system_status_unpack(&dst, frame.data, frame.can_dlc) != 0) {
      return;
    }

    mirena_common::msg::AsInfo out;
    out.status = map_as_status(dst.as_status);
    out.mission = map_ami_mission(dst.ami_state);
    as_info_pub_->publish(out);
  }

  // --- CAN WheelInfo -> ROS WheelSpeeds ---
  //
  // WheelInfo carries the two driven wheels, which is all this car instruments:
  // rl_rpm and rr_rpm, signed int16 in raw RPM. The generated decode is a plain
  // cast (scale 1, offset 0), so the sign survives and reverse rotation is a
  // negative speed rather than a wrapped one.
  //
  // Published as RPM, exactly as the bus reports it. Converting to m/s here
  // would bake a rolling radius into the driver, and that is a tyre property --
  // it moves with compound, pressure and wear -- so it belongs to whoever fuses
  // the measurement, not to the layer that reads the wire.
  void handle_wheel_info(const struct can_frame & frame)
  {
    struct ter_wheel_info_t dst;
    if (ter_wheel_info_unpack(&dst, frame.data, frame.can_dlc) != 0) {
      return;
    }

    mirena_common::msg::WheelSpeeds out;
    // Stamped on arrival. The bus carries no timestamp of its own, so this is
    // the earliest moment the sample can be attributed to; the polling delay it
    // hides is bounded by the 5 ms rx timer.
    out.header.stamp = now();
    out.header.frame_id = kWheelFrame;
    out.rl = ter_wheel_info_rl_rpm_decode(dst.rl_rpm);
    out.rr = ter_wheel_info_rr_rpm_decode(dst.rr_rpm);
    wheel_speeds_pub_->publish(out);
  }

  // --- CAN APPS / BPPS / STEER -> ROS CarControl ---
  //
  // These are the manual driver's inputs, not our commands: the accelerator and
  // brake pedals and the steering sensor, all broadcast by the pedal ECU. They are
  // published so a manual test can be recorded and replayed as a ROS bag, which is
  // the only way to compare what a human asked the car for against what the car
  // did -- the CAN log has the commands, this has them timestamped in ROS time.

  // Accelerator pedal. APPS_AV is the ECU's own average of the two pedal sensors,
  // which is the signal to trust: one pot failing is a fault the ECU reports, not
  // something to reconstruct here.
  void handle_apps(const struct can_frame & frame)
  {
    struct ter_apps_t dst;
    if (ter_apps_unpack(&dst, frame.data, frame.can_dlc) != 0) {
      return;
    }

    gas_apps_ = std::clamp(ter_apps_apps_av_decode(dst.apps_av) / 255.0, 0.0, 1.0);
    publish_pedal_control();
  }

  // Brake pedal, as pressure. Normalized against a full-scale that is a parameter
  // rather than a constant because the DBC's 1 Bar is a nominal range, not a
  // measured full pedal, and getting it wrong scales every braking sample in the bag.
  void handle_bpps(const struct can_frame & frame)
  {
    struct ter_bpps_t dst;
    if (ter_bpps_unpack(&dst, frame.data, frame.can_dlc) != 0) {
      return;
    }

    const double full_scale = (bpps_full_scale_bar_ > 0.0) ? bpps_full_scale_bar_ : 1.0;
    gas_brake_ = std::clamp(ter_bpps_bpps_decode(dst.bpps) / full_scale, 0.0, 1.0);
    publish_pedal_control();
  }

  // Steering sensor, published exactly as the bus reports it. The signal is in
  // Grad, and CarControl.steer_angle is documented in radians; converting here
  // would silently disagree with the CAN log recorded from the same bus in the same
  // session, and the conversion factor belongs to whoever reads the bag.
  void handle_steer(const struct can_frame & frame)
  {
    struct ter_steer_t dst;
    if (ter_steer_unpack(&dst, frame.data, frame.can_dlc) != 0) {
      return;
    }

    steer_ = ter_steer_angle_decode(dst.angle);
    publish_pedal_control();
  }

  // Publishes the latest known pedal state. Called on every pedal frame, so gas and
  // steering are latched: whichever of them did not change in this frame keeps its
  // previous value rather than dropping to zero and back.
  void publish_pedal_control()
  {
    mirena_common::msg::CarControl out;
    // Stamped on arrival, like the wheel speeds: the bus carries no timestamp, so
    // this is the earliest moment the sample can be attributed to, and the polling
    // delay it hides is bounded by the 5 ms rx timer. No frame_id -- a pedal command
    // is not expressed in any of this workspace's frames, and the one on /control
    // leaves it to whoever is commanding.
    out.header.stamp = now();
    out.gas = std::clamp(gas_apps_ - gas_brake_, -1.0, 1.0);
    out.steer_angle = steer_;
    car_control_pub_->publish(out);
  }

  // DBC AS_status choices use a different enumeration than the ROS AsInfo status constants.
  static uint8_t map_as_status(uint8_t dbc_value)
  {
    using Msg = mirena_common::msg::AsInfo;
    switch (dbc_value) {
      case TER_DV_SYSTEM_STATUS_AS_STATUS_AS_STATUS_OFF_CHOICE:       return Msg::AS_STATUS_OFF;
      case TER_DV_SYSTEM_STATUS_AS_STATUS_AS_STATUS_READY_CHOICE:     return Msg::AS_STATUS_READY;
      case TER_DV_SYSTEM_STATUS_AS_STATUS_AS_STATUS_DRIVING_CHOICE:   return Msg::AS_STATUS_DRIVING;
      case TER_DV_SYSTEM_STATUS_AS_STATUS_AS_STATUS_FINISHED_CHOICE:  return Msg::AS_STATUS_FINISHED;
      case TER_DV_SYSTEM_STATUS_AS_STATUS_AS_STATUS_EMERGENCY_CHOICE: return Msg::AS_STATUS_EMERGENCY;
      default:                                                        return Msg::AS_STATUS_OFF;
    }
  }

  // ROS MissionInfo status maps onto the DBC DV_info MISSION_STATUS choices.
  // DV_info has no FAILED choice; both terminal states (FINISHED/FAILED) map to
  // FINISHED, and any unknown value falls back to NONE.
  static uint8_t map_mission_status(uint8_t status)
  {
    using Msg = mirena_common::msg::MissionInfo;
    switch (status) {
      case Msg::MISSION_STATUS_INACTIVE:    return TER_DV_INFO_MISSION_STATUS_NONE_CHOICE;
      case Msg::MISSION_STATUS_CONFIGURING: return TER_DV_INFO_MISSION_STATUS_SELECTED_CHOICE;
      case Msg::MISSION_STATUS_READY:       return TER_DV_INFO_MISSION_STATUS_READY_CHOICE;
      case Msg::MISSION_STATUS_RUNNING:     return TER_DV_INFO_MISSION_STATUS_RUNNING_CHOICE;
      case Msg::MISSION_STATUS_FINISHED:    return TER_DV_INFO_MISSION_STATUS_FINISHED_CHOICE;
      case Msg::MISSION_STATUS_FAILED:      return TER_DV_INFO_MISSION_STATUS_FAILED_CHOICE;
      default:                              return TER_DV_INFO_MISSION_STATUS_NONE_CHOICE;
    }
  }

  // DBC AMI_state choices map onto the ROS AsInfo mission constants.
  static uint8_t map_ami_mission(uint8_t ami_state)
  {
    using Msg = mirena_common::msg::AsInfo;
    switch (ami_state) {
      case TER_DV_SYSTEM_STATUS_AMI_STATE_AMI_STATE_ACCELERATION_CHOICE: return Msg::AS_MISSION_ACCELERATION;
      case TER_DV_SYSTEM_STATUS_AMI_STATE_AMI_STATE_SKIDPAD_CHOICE:      return Msg::AS_MISSION_SKIDPAD;
      case TER_DV_SYSTEM_STATUS_AMI_STATE_AMI_STATE_TRACKDRIVE_CHOICE:   return Msg::AS_MISSION_TRACKDRIVE;
      case TER_DV_SYSTEM_STATUS_AMI_STATE_AMI_STATE_BRAKETEST_CHOICE:    return Msg::AS_MISSION_BRAKE_TEST;
      case TER_DV_SYSTEM_STATUS_AMI_STATE_AMI_STATE_INSPECTION_CHOICE:   return Msg::AS_MISSION_INSPECTION;
      case TER_DV_SYSTEM_STATUS_AMI_STATE_AMI_STATE_AUTOCROSS_CHOICE:    return Msg::AS_MISSION_AUTOCROSS;
      default:                                                          return Msg::AS_MISSION_NONE;
    }
  }

  std::string iface_;
  int can_fd_ {-1};
  double bpps_full_scale_bar_ {1.0};
  // Latest manual (pedal) input, latched across frames. See publish_pedal_control.
  double gas_apps_ {0.0};
  double gas_brake_ {0.0};
  double steer_ {0.0};
  /// Null unless the `log` parameter asked for recording. Its presence is also
  /// what decides whether the socket runs filtered.
  std::unique_ptr<mirena::CanLogger> logger_;
  rclcpp::Subscription<mirena_common::msg::CarControl>::SharedPtr control_sub_;
  rclcpp::Subscription<mirena_common::msg::MissionInfo>::SharedPtr mission_info_sub_;
  rclcpp::Publisher<mirena_common::msg::AsInfo>::SharedPtr as_info_pub_;
  rclcpp::Publisher<mirena_common::msg::WheelSpeeds>::SharedPtr wheel_speeds_pub_;
  rclcpp::Publisher<mirena_common::msg::CarControl>::SharedPtr car_control_pub_;
  rclcpp::TimerBase::SharedPtr rx_timer_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<MirenaCan>());
  rclcpp::shutdown();
  return 0;
}
