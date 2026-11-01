# Future Telemetry Instrumentation & Data Acquisition Requirements
## Full-Observability Telemetry Specification for MirenaSim Hardware-in-the-Loop & Replay

**Author**: Sensor & Instrumentation Engineering Specialist Group  
**Target Vehicle Platform**: Formula Student TeR26 / TeR27 Electric  
**Document ID**: `FS-DAQ-REQ-2026-V1`  
**Date**: November 2026  

---

## 1. Objective & Scope

The Phase III audit and Phase IV observability forensics proved that the primary source of open-loop speed tracking residual drift ($+1.236\text{ m/s}$ session bias) was the absence of hydraulic brake pressure instrumentation in the recorded telemetry.

This document defines the complete sensor, CAN bus, and telemetry logging architecture required to achieve **100% observability** during future track testing campaigns, eliminating all input reconstruction ambiguities for simulator replay and correlation.

---

## 2. Sensor Instrumentation Specifications

### Table 2.1: Required Telemetry Transducers and Measurement Specifications

| Subsystem Channel | Recommended Sensor Hardware | Measurement Range | Required Precision / Accuracy | Minimum Logging Rate | CAN / Signal Interface |
| :--- | :--- | :---: | :---: | :---: | :--- |
| **Front Brake Line Pressure** | Sensata 114CP / Bosch Motorsport PST-F 1 | $0\text{ to }100.0\text{ bar}$ | $\pm 0.5\%\text{ FSO}$ | $100.0\text{ Hz}$ | $0.5\text{--}4.5\text{ V}$ analog to MoTeC / CAN node |
| **Rear Brake Line Pressure** | Sensata 114CP / Bosch Motorsport PST-F 1 | $0\text{ to }100.0\text{ bar}$ | $\pm 0.5\%\text{ FSO}$ | $100.0\text{ Hz}$ | $0.5\text{--}4.5\text{ V}$ analog to MoTeC / CAN node |
| **Brake Pedal Travel** | Penny & Giles SLS095 Linear Potentiometer | $0\text{ to }40.0\text{ mm}$ | $\pm 0.2\%\text{ FSO}$ | $100.0\text{ Hz}$ | $0\text{--}5.0\text{ V}$ analog to CAN node |
| **APPS Accelerator Pedal** | Dual redundant rotary Hall effect (CTS 288) | $0\text{ to }100.0\%$ | $\pm 0.5\%\text{ FSO}$ | $100.0\text{ Hz}$ | Dual analog / CAN (EV rule compliant) |
| **Steering Wheel Angle** | Vishay 157 conductive plastic rotary sensor | $\pm 180.0^\circ$ | $\pm 0.25^\circ$ | $100.0\text{ Hz}$ | SPI / CAN encoder |
| **Inverter Delivered Torque** | EMRAX / Bamocar D3 CAN internal state | $\pm 150.0\text{ N}\cdot\text{m}$ | $\pm 1.0\text{ N}\cdot\text{m}$ | $100.0\text{ Hz}$ | CAN ID `0x201` (Inverter Status) |
| **Inverter Phase Current** | Bamocar D3 internal Hall sensor array | $\pm 350.0\text{ Arms}$ | $\pm 1.5\text{ Arms}$ | $100.0\text{ Hz}$ | CAN ID `0x202` (Inverter Current) |
| **Traction Accumulator Voltage** | BMS Master LEM LV-25-P transducer | $0\text{ to }450.0\text{ V}$ | $\pm 0.5\text{ V}$ | $50.0\text{ Hz}$ | CAN ID `0x301` (BMS Status) |
| **4x Wheel Speeds (FL, FR, RL, RR)** | Honeyell GT101 Hall gear tooth sensors (48-tooth tone ring) | $0\text{ to }1500.0\text{ RPM}$ | $\pm 0.25\text{ RPM}$ | $100.0\text{ Hz}$ | Digital frequency / CAN wheel hub node |
| **6-DOF IMU Accelerations** | VectorNav VN-100 Rugged Inertial Sensor | $\pm 16.0\text{ g}$ | $\pm 0.01\text{ m/s}^2$ | $200.0\text{ Hz}$ | RS-422 / CAN (`/imu/data_raw`) |
| **6-DOF IMU Angular Rates** | VectorNav VN-100 Rugged Inertial Sensor | $\pm 2000.0^\circ/\text{s}$ | $\pm 0.01^\circ/\text{s}$ | $200.0\text{ Hz}$ | RS-422 / CAN (`/imu/data_raw`) |
| **4x Suspension Travel** | Penny & Giles SLS130 linear potentiometers | $0\text{ to }60.0\text{ mm}$ | $\pm 0.1\text{ mm}$ | $100.0\text{ Hz}$ | Analog to CAN node |

---

## 3. CAN Bus Network Architecture & Timing Budgets

To prevent CAN bus arbitration delays, clock jitter, and dropped frames, telemetry messages must be assigned strictly prioritized standard 11-bit CAN IDs on a dedicated **$1.0\text{ Mbps}$ Motorsport Telemetry CAN Bus**:

### Table 3.1: CAN Identifier Mapping and Message Payload Layout

| CAN ID (Hex) | Priority | Message Name | Payload Contents (8 Bytes) | Transmission Period | Bus Load Contribution |
| :--- | :---: | :--- | :--- | :---: | :---: |
| `0x080` | Highest | `CAN_SYNC_TIME` | Unix Epoch timestamp (uint64_t nanoseconds) | $100.0\text{ Hz}$ ($10\text{ ms}$) | $1.28\%$ |
| `0x100` | High | `IMU_ACCEL` | `ax` (int16, 0.001g), `ay` (int16), `az` (int16), status (uint16) | $200.0\text{ Hz}$ ($5\text{ ms}$) | $2.56\%$ |
| `0x101` | High | `IMU_GYRO` | `gx` (int16, 0.01 deg/s), `gy` (int16), `gz` (int16), temp (int16) | $200.0\text{ Hz}$ ($5\text{ ms}$) | $2.56\%$ |
| `0x150` | Medium-High | `BRAKE_HYDRAULIC` | `p_front_bar` (uint16, 0.01 bar), `p_rear_bar` (uint16), pedal_pct (uint16), status | $100.0\text{ Hz}$ ($10\text{ ms}$) | $1.28\%$ |
| `0x160` | Medium | `DRIVER_CONTROLS` | `gas_apps_pct` (uint16, 0.01%), `steer_angle_rad` (int16, 0.0001 rad), flags | $100.0\text{ Hz}$ ($10\text{ ms}$) | $1.28\%$ |
| `0x170` | Medium | `WHEEL_SPEEDS_FRONT`| `fl_rpm` (uint16, 0.1 RPM), `fr_rpm` (uint16), slip_fl (int16), slip_fr (int16) | $100.0\text{ Hz}$ ($10\text{ ms}$) | $1.28\%$ |
| `0x171` | Medium | `WHEEL_SPEEDS_REAR` | `rl_rpm` (uint16, 0.1 RPM), `rr_rpm` (uint16), slip_rl (int16), slip_rr (int16) | $100.0\text{ Hz}$ ($10\text{ ms}$) | $1.28\%$ |
| `0x200` | Normal | `MOTOR_DYNAMICS` | `torque_delivered_nm` (int16, 0.1 Nm), `motor_rpm` (int16), `dc_bus_v` (uint16) | $100.0\text{ Hz}$ ($10\text{ ms}$) | $1.28\%$ |
| `0x220` | Normal | `SUSPENSION_TRAVEL` | `damper_fl_mm` (int16, 0.01mm), `fr` (int16), `rl` (int16), `rr` (int16) | $100.0\text{ Hz}$ ($10\text{ ms}$) | $1.28\%$ |

**Total Cumulative Bus Load at $1.0\text{ Mbps}$**: **$14.08\%$** (well below the $40.0\%$ stability limit, guaranteeing zero frame dropouts or queue latency).

---

## 4. ROS 2 Telemetry Message Definitions

For integration into the MirenaSim replay and CorrelationLab pipeline, the following standardized ROS 2 message definitions must be added to `mirena_common`:

### `mirena_common/msg/BrakeTelemetry.msg`
```ros2
std_msgs/Header header

# Hydraulic brake line pressure in Bar (gauge)
float32 front_pressure_bar
float32 rear_pressure_bar

# Normalized brake pedal position [0.0, 1.0]
float32 pedal_travel_pct

# Mechanical brake balance (% front)
float32 front_bias_fraction

# Emergency Brake System (EBS) active flag
bool ebs_triggered
```

### `mirena_common/msg/PowertrainTelemetry.msg`
```ros2
std_msgs/Header header

# Delivered motor shaft torque in N*m (positive = drive, negative = regen)
float32 delivered_torque_nm
float32 requested_torque_nm

# Motor rotor speed in RPM
float32 motor_rpm

# Inverter electrical parameters
float32 dc_bus_voltage_v
float32 phase_current_arms
float32 inverter_temp_c
float32 motor_temp_c
```

---

## 5. Implementation Roadmap for TeR26 / TeR27

1. **Step 1 (Hardware Installation)**:
   Install two Sensata 114CP pressure transducers directly into the master cylinder outlet ports (front and rear brake circuits).
2. **Step 2 (DAQ Calibration)**:
   Perform 5-point calibration on a hydraulic bench press ($0, 20, 40, 60, 80\text{ bar}$) with calibration certificate recorded in `CorrelationLab/config/sensors/`.
3. **Step 3 (Firmware Ingestion)**:
   Configure Teensy 4.1 CAN gateway node to publish CAN ID `0x150` onto the main telemetry bus.
4. **Step 4 (ROS 2 Logging)**:
   Add `/telemetry/brakes` and `/telemetry/powertrain` to `ros2 bag record` launch script.
5. **Step 5 (Simulator Replay Ingestion)**:
   Connect `front_pressure_bar` and `rear_pressure_bar` directly to `BrakeSystem.gd` in `run_modular_replay.gd`, achieving 100% physically deterministic open-loop deceleration.
