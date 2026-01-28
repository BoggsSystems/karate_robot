# Sensei Robot Hardware Architecture

## Overview

This document defines the hardware architecture for the physical Sensei karate robot, bridging from simulation to real-world deployment.

## Robot Specifications

| Attribute | Value |
|-----------|-------|
| Height | ~30 cm (1 foot) |
| DOF | 14 (3+3 arms, 3+3 legs, 2 head) |
| Weight Target | < 2 kg |
| Operating Time | 30+ minutes |

---

## Compute Architecture

### Three-Tier Design

```
┌─────────────────┐
│      BRAIN      │  Policy inference, planning, vision
│   (Linux SBC)   │  ROS2 nodes, trained policies
│    100-500ms    │  
└────────┬────────┘
         │ UART/USB (1Mbps)
         │
┌────────┴────────┐
│      SPINE      │  Real-time control, safety
│   (Teensy/STM)  │  1kHz control loop
│      1ms        │  Sensor fusion, reflex
└────────┬────────┘
         │ I2C/SPI/Serial
         │
┌────────┴────────┐
│   PERIPHERALS   │  Motors, encoders, sensors
│   (Servos/IMU)  │  Direct hardware interface
└─────────────────┘
```

### Brain Options

| Option | Pros | Cons | Cost |
|--------|------|------|------|
| **Raspberry Pi 5** | Cheap, ROS2 ready, community | Limited compute | $80 |
| **Jetson Orin Nano** | GPU, AI inference, powerful | Power hungry, complex | $500 |
| **Intel NUC** | x86 compatible, powerful | Large, expensive | $400 |
| **Orange Pi 5** | Good value, RK3588 | Less support | $100 |

**Recommendation**: Start with **Raspberry Pi 5** for initial bring-up, upgrade to **Jetson** if vision/ML needed.

### Spine MCU Options

| Option | Pros | Cons | Cost |
|--------|------|------|------|
| **Teensy 4.1** | Fast (600MHz), Arduino compatible | Limited I/O | $30 |
| **ESP32-S3** | WiFi, cheap, dual-core | Less deterministic | $10 |
| **STM32H7** | Industrial, deterministic | Harder to program | $25 |

**Recommendation**: **Teensy 4.1** for prototyping, **STM32H7** for production.

---

## Actuators

### Servo Options for 14 DOF

| Servo | Torque | Speed | Feedback | Cost/ea | Notes |
|-------|--------|-------|----------|---------|-------|
| **Dynamixel XL330** | 0.6 Nm | 0.1s/60° | Pos/Vel/Load | $20 | Budget, good for small robot |
| **Dynamixel XL430** | 1.4 Nm | 0.1s/60° | Pos/Vel/Load | $50 | Better torque |
| **Dynamixel XM430** | 2.7 Nm | 0.1s/60° | Pos/Vel/Load | $60 | Recommended |
| **Feetech STS3215** | 1.5 Nm | 0.1s/60° | Pos only | $25 | Cheap alternative |
| **Custom BLDC** | Variable | Fast | Full state | $100+ | Advanced |

**Recommendation**: **Dynamixel XL430 or XM430** - smart servos with built-in feedback and daisy-chain communication.

### Joint Allocation

| Joint | Torque Needed | Servo Choice |
|-------|---------------|--------------|
| Shoulder Pan | Medium | XL430 |
| Shoulder Lift | High | XM430 |
| Elbow | Medium | XL430 |
| Hip Pitch | High | XM430 |
| Knee Pitch | High | XM430 |
| Ankle Pitch | High | XM430 |
| Neck Pan/Tilt | Low | XL330 |

---

## Sensors

### Required Sensors

| Sensor | Purpose | Options | Cost |
|--------|---------|---------|------|
| **IMU** | Balance, orientation | BNO085, MPU6050 | $15-50 |
| **Foot Pressure** | Ground contact | FSR402, load cells | $20-100 |
| **Joint Position** | Feedback (if not smart servo) | AS5600, encoders | Built-in |
| **Current Sensing** | Safety, collision | INA219 | $5 |

### Optional Sensors

| Sensor | Purpose | Options | Cost |
|--------|---------|---------|------|
| **Camera** | Vision, tracking | OAK-D, Pi Camera | $50-200 |
| **Distance** | Obstacle detection | VL53L1X ToF | $15 |
| **Microphone** | Voice commands | INMP441 | $5 |

---

## Power System

### Battery Selection

| Robot Size | Battery | Voltage | Capacity | Runtime |
|------------|---------|---------|----------|---------|
| Desktop (30cm) | 2S LiPo | 7.4V | 2000mAh | ~45 min |
| Medium (60cm) | 3S LiPo | 11.1V | 3000mAh | ~30 min |
| Full-size | 4S LiPo | 14.8V | 5000mAh | ~30 min |

### Power Distribution

```
Battery (2S/3S LiPo)
    │
    ├── BMS (protection)
    │
    ├── 5V Buck → Raspberry Pi, sensors
    │
    ├── 6-8V → Servos (direct from battery)
    │
    └── 3.3V LDO → MCU, logic
```

### Safety Features

- Over-current protection per servo
- Under-voltage cutoff (protect LiPo)
- E-stop button (physical kill switch)
- Software watchdog timer

---

## Communication Buses

### Recommended Setup

```
Raspberry Pi 5
    │
    ├── USB ──────────► Teensy 4.1 (1Mbps UART)
    │                       │
    │                       ├── UART ────► Dynamixel chain (1Mbps)
    │                       │
    │                       ├── I2C ─────► IMU (BNO085)
    │                       │
    │                       └── ADC ─────► Foot sensors
    │
    └── USB ──────────► Camera (optional)
```

### Protocol Stack

| Layer | Protocol | Speed |
|-------|----------|-------|
| Brain ↔ Spine | Custom binary over UART | 1 Mbps |
| Spine ↔ Servos | Dynamixel Protocol 2.0 | 1 Mbps |
| Spine ↔ IMU | I2C | 400 kHz |
| ROS2 Layer | DDS (internal) | N/A |

---

## Frame/Mechanical

### Materials

| Part | Material | Method |
|------|----------|--------|
| Torso | Aluminum or 3D print | CNC or FDM |
| Limbs | 3D printed PETG | FDM |
| Joints | Aluminum brackets | Laser cut |
| Feet | TPU (flexible) | FDM |

### 3D Printing Guidelines

- Layer height: 0.2mm for structural, 0.1mm for detail
- Infill: 30-50% for limbs, 80%+ for load-bearing
- Material: PETG (good strength/flex balance)

---

## Bill of Materials (Recommended Build)

### Tier 1: Minimum Viable Robot (~$600)

| Item | Qty | Unit Cost | Total |
|------|-----|-----------|-------|
| Raspberry Pi 5 (8GB) | 1 | $80 | $80 |
| Teensy 4.1 | 1 | $30 | $30 |
| Dynamixel XL430-W250 | 14 | $50 | $700 |
| Dynamixel U2D2 (interface) | 1 | $35 | $35 |
| BNO085 IMU | 1 | $20 | $20 |
| FSR402 (foot sensors) | 4 | $5 | $20 |
| 2S LiPo 2200mAh | 1 | $25 | $25 |
| Power distribution board | 1 | $30 | $30 |
| 3D printing filament | 2kg | $25 | $50 |
| Hardware (screws, wires) | - | - | $50 |
| **TOTAL** | | | **~$1,040** |

### Tier 2: Performance Robot (~$2,000)

Add to Tier 1:
- Upgrade servos to XM430 (+$140)
- Jetson Orin Nano instead of Pi (+$420)
- OAK-D camera (+$150)
- Custom PCB (+$100)
- Aluminum frame parts (+$150)

---

## Software Integration

### ROS2 Hardware Interface

The trained policy connects to real hardware via ros2_control:

```yaml
# ros2_controllers.yaml (real hardware)
controller_manager:
  ros__parameters:
    update_rate: 100  # Hz

joint_state_broadcaster:
  type: joint_state_broadcaster/JointStateBroadcaster

position_controller:
  type: position_controllers/JointGroupPositionController
  joints:
    - l_shoulder_pan
    - l_shoulder_lift
    - l_elbow_flex
    # ... all 14 joints
```

### Hardware Bridge Node

```python
# Bridges ROS2 commands to Teensy/Dynamixel
class HardwareBridgeNode(Node):
    def __init__(self):
        # Subscribe to joint commands
        self.cmd_sub = self.create_subscription(
            Float64MultiArray, 'joint_commands', self.on_command)
        
        # Publish joint states
        self.state_pub = self.create_publisher(
            JointState, 'joint_states', 10)
        
        # Serial connection to Teensy
        self.serial = serial.Serial('/dev/ttyACM0', 1000000)
```

---

## Development Phases

### Phase 1: Single Limb Test ($200)
- 3 servos (one arm)
- Teensy 4.1
- Validate control loop

### Phase 2: Upper Body ($500)
- Both arms + head (8 DOF)
- Add IMU
- Test balance sensing

### Phase 3: Full Robot ($1,000)
- Add legs (14 DOF total)
- Add foot sensors
- Basic standing

### Phase 4: Autonomous ($1,500+)
- Add Raspberry Pi / Jetson
- Deploy trained policy
- Walking tests

---

## Safety Considerations

1. **Current limiting**: Set servo max current below stall
2. **Watchdog timer**: MCU resets if brain stops responding
3. **E-stop**: Physical button cuts motor power
4. **Soft limits**: Software joint limits before mechanical stops
5. **Fall detection**: IMU triggers safe shutdown on large angles

---

## Next Steps

1. [ ] Order Tier 1 components
2. [ ] Design and print frame
3. [ ] Assemble single arm for testing
4. [ ] Develop Teensy firmware
5. [ ] Create ROS2 hardware interface
6. [ ] Test with trained policy

---

*Last updated: January 28, 2026*
