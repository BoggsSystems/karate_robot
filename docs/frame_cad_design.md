# Sensei Robot Frame & CAD Design Plan

## Overview

This document provides detailed specifications for designing and 3D printing the Sensei robot frame, integrating Dynamixel XL430 servos.

---

## Robot Dimensions (From URDF)

### Overall Specifications

| Attribute | Value |
|-----------|-------|
| **Total Height** | ~300mm (12 inches) standing |
| **Torso Width** | 200mm |
| **Torso Depth** | 140mm |
| **Shoulder Span** | ~280mm |
| **Hip Width** | 120mm |
| **Weight Target** | < 2kg |

### Dynamixel XL430-W250 Dimensions

```
┌─────────────────────────────────────┐
│     DYNAMIXEL XL430-W250-T          │
├─────────────────────────────────────┤
│  Dimensions: 28.5 x 46.5 x 34 mm    │
│  Weight: 57.2g                      │
│  Stall Torque: 1.4 Nm               │
│  No-load Speed: 57 RPM              │
│  Input Voltage: 6.5-12V             │
│                                     │
│       ┌─────────────┐               │
│       │    28.5mm   │               │
│       │  ┌───────┐  │               │
│  34mm │  │ SERVO │  │ 46.5mm        │
│       │  │       │  │               │
│       │  └───────┘  │               │
│       └─────────────┘               │
│                                     │
│  Mounting: M2.5 screws              │
│  Horn: 4x M2 mounting holes         │
└─────────────────────────────────────┘
```

---

## Component Breakdown

### Exploded View

```
                    ┌─────┐
                    │HEAD │ ← Neck Pan/Tilt (2 servos)
                    └──┬──┘
                       │
        ┌──────────────┼──────────────┐
        │              │              │
   ┌────┴────┐    ┌────┴────┐    ┌────┴────┐
   │L_SHOULDER│    │  TORSO  │    │R_SHOULDER│
   │(servo)  │    │         │    │(servo)   │
   └────┬────┘    │ ┌─────┐ │    └────┬─────┘
        │         │ │ Pi5 │ │         │
   ┌────┴────┐    │ ├─────┤ │    ┌────┴────┐
   │L_UPPER  │    │ │Teensy│ │    │R_UPPER  │
   │ARM      │    │ ├─────┤ │    │ARM      │
   │(servo)  │    │ │LiPo │ │    │(servo)  │
   └────┬────┘    │ └─────┘ │    └────┬────┘
        │         └────┬────┘         │
   ┌────┴────┐         │         ┌────┴────┐
   │L_FOREARM│    ┌────┴────┐    │R_FOREARM│
   │(servo)  │    │  PELVIS │    │(servo)  │
   └────┬────┘    └────┬────┘    └────┬────┘
        │         ┌────┴────┐         │
   ┌────┴────┐    │         │    ┌────┴────┐
   │L_HAND   │┌───┴───┐ ┌───┴───┐│R_HAND   │
   └─────────┘│L_HIP  │ │R_HIP  │└─────────┘
              │(servo)│ │(servo)│
              └───┬───┘ └───┬───┘
              ┌───┴───┐ ┌───┴───┐
              │L_KNEE │ │R_KNEE │
              │(servo)│ │(servo)│
              └───┬───┘ └───┬───┘
              ┌───┴───┐ ┌───┴───┐
              │L_ANKLE│ │R_ANKLE│
              │(servo)│ │(servo)│
              └───┬───┘ └───┬───┘
              ┌───┴───┐ ┌───┴───┐
              │L_FOOT │ │R_FOOT │
              │(TPU)  │ │(TPU)  │
              └───────┘ └───────┘
```

---

## Part Specifications

### 1. Torso Assembly

```
┌─────────────────────────────────────────────────────────────┐
│                      TORSO (TOP VIEW)                       │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│    ┌─────────────────────────────────────────────────┐     │
│    │                   200mm                          │     │
│    │  ┌─────┐                             ┌─────┐    │     │
│    │  │L_SH │                             │R_SH │    │140mm│
│    │  │SERVO│    ┌─────────────────┐      │SERVO│    │     │
│    │  └─────┘    │                 │      └─────┘    │     │
│    │             │   ELECTRONICS   │                 │     │
│    │             │   CAVITY        │                 │     │
│    │             │                 │                 │     │
│    │             │  Pi5: 85x56mm   │                 │     │
│    │             │  Teensy: 60x18  │                 │     │
│    │             │  LiPo: 105x35   │                 │     │
│    │             │                 │                 │     │
│    │             └─────────────────┘                 │     │
│    │                                                 │     │
│    └─────────────────────────────────────────────────┘     │
│                                                             │
└─────────────────────────────────────────────────────────────┘

TORSO SPECIFICATIONS:
- Material: PETG
- Wall thickness: 3mm
- Infill: 40%
- Print time: ~8-10 hours
- Features:
  - Shoulder servo mounts (angled 15° outward)
  - Electronics bay (100x80x40mm internal)
  - Cable routing channels
  - Ventilation slots
  - Battery access door
```

#### Torso Parts List

| Part | Dimensions | Print Time | Qty |
|------|------------|------------|-----|
| torso_main.stl | 200x140x100mm | 8 hrs | 1 |
| torso_back_cover.stl | 180x120x3mm | 1 hr | 1 |
| battery_door.stl | 80x40x3mm | 20 min | 1 |
| shoulder_mount_L.stl | 50x40x35mm | 1.5 hrs | 1 |
| shoulder_mount_R.stl | 50x40x35mm | 1.5 hrs | 1 |

---

### 2. Arm Assembly (x2)

```
┌─────────────────────────────────────────────────────────────┐
│                    LEFT ARM (SIDE VIEW)                     │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│         ┌───────────┐                                       │
│         │ SHOULDER  │ ← Servo 1: Pan (Z-axis rotation)      │
│         │  JOINT    │   Mounted in torso                    │
│         └─────┬─────┘                                       │
│               │                                             │
│         ┌─────┴─────┐                                       │
│         │ SHOULDER  │ ← Servo 2: Lift (Y-axis rotation)     │
│         │  SERVO    │   XL430                               │
│         └─────┬─────┘                                       │
│               │                                             │
│         ┌─────┴─────┐                                       │
│         │           │                                       │
│         │  UPPER    │   120mm length                        │
│         │   ARM     │   25mm diameter tube                  │
│         │           │                                       │
│         └─────┬─────┘                                       │
│               │                                             │
│         ┌─────┴─────┐                                       │
│         │  ELBOW    │ ← Servo 3: Flex (Y-axis rotation)     │
│         │  SERVO    │   XL430                               │
│         └─────┬─────┘                                       │
│               │                                             │
│         ┌─────┴─────┐                                       │
│         │           │                                       │
│         │ FOREARM   │   110mm length                        │
│         │           │   22mm diameter tube                  │
│         └─────┬─────┘                                       │
│               │                                             │
│         ┌─────┴─────┐                                       │
│         │   HAND    │   60x40x30mm                          │
│         │   (fist)  │   Fixed attachment                    │
│         └───────────┘                                       │
│                                                             │
│  Total arm length: ~280mm                                   │
│  Weight: ~250g per arm                                      │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

#### Arm Parts List (per arm)

| Part | Dimensions | Print Time | Qty |
|------|------------|------------|-----|
| shoulder_bracket.stl | 50x50x40mm | 2 hrs | 1 |
| upper_arm_tube.stl | 25x25x120mm | 1.5 hrs | 1 |
| elbow_bracket.stl | 45x45x35mm | 1.5 hrs | 1 |
| forearm_tube.stl | 22x22x110mm | 1.5 hrs | 1 |
| hand_fist.stl | 60x40x30mm | 1 hr | 1 |

---

### 3. Leg Assembly (x2)

```
┌─────────────────────────────────────────────────────────────┐
│                    LEFT LEG (FRONT VIEW)                    │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│         ┌───────────┐                                       │
│         │   HIP     │ ← Servo 4: Pitch (Y-axis rotation)    │
│         │  SERVO    │   XL430 - highest torque needed       │
│         └─────┬─────┘                                       │
│               │                                             │
│         ┌─────┴─────┐                                       │
│         │   HIP     │   Hip bracket                         │
│         │ BRACKET   │   Connects servo to thigh             │
│         └─────┬─────┘                                       │
│               │                                             │
│         ┌─────┴─────┐                                       │
│         │           │                                       │
│         │  THIGH    │   140mm length                        │
│         │           │   30mm diameter tube                  │
│         │           │                                       │
│         └─────┬─────┘                                       │
│               │                                             │
│         ┌─────┴─────┐                                       │
│         │  KNEE     │ ← Servo 5: Pitch (Y-axis rotation)    │
│         │  SERVO    │   XL430 - high torque needed          │
│         └─────┬─────┘                                       │
│               │                                             │
│         ┌─────┴─────┐                                       │
│         │           │                                       │
│         │  SHIN     │   120mm length                        │
│         │           │   28mm diameter tube                  │
│         └─────┬─────┘                                       │
│               │                                             │
│         ┌─────┴─────┐                                       │
│         │  ANKLE    │ ← Servo 6: Pitch (Y-axis rotation)    │
│         │  SERVO    │   XL430                               │
│         └─────┬─────┘                                       │
│               │                                             │
│     ┌─────────┴─────────┐                                   │
│     │       FOOT        │   100x60x30mm                     │
│     │    ┌───┐  ┌───┐   │   TPU for grip                    │
│     │    │FSR│  │FSR│   │   2x FSR sensors                  │
│     └────┴───┴──┴───┴───┘                                   │
│                                                             │
│  Total leg length: ~320mm                                   │
│  Weight: ~400g per leg                                      │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

#### Leg Parts List (per leg)

| Part | Dimensions | Print Time | Qty |
|------|------------|------------|-----|
| hip_mount.stl | 60x50x45mm | 2 hrs | 1 |
| hip_bracket.stl | 50x50x40mm | 1.5 hrs | 1 |
| thigh_tube.stl | 30x30x140mm | 2 hrs | 1 |
| knee_bracket.stl | 55x50x40mm | 2 hrs | 1 |
| shin_tube.stl | 28x28x120mm | 1.5 hrs | 1 |
| ankle_bracket.stl | 50x45x35mm | 1.5 hrs | 1 |
| foot_base.stl (TPU) | 100x60x20mm | 2 hrs | 1 |
| foot_sole.stl (TPU) | 100x60x10mm | 1 hr | 1 |

---

### 4. Head Assembly

```
┌─────────────────────────────────────────────────────────────┐
│                      HEAD ASSEMBLY                          │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│              ┌─────────────────┐                            │
│              │                 │                            │
│              │      HEAD       │   60x60x50mm               │
│              │     SHELL       │   Contains camera mount    │
│              │                 │                            │
│              │    ┌─────┐      │                            │
│              │    │ CAM │      │   Optional: OAK-D Lite     │
│              │    └─────┘      │                            │
│              └────────┬────────┘                            │
│                       │                                     │
│              ┌────────┴────────┐                            │
│              │   NECK TILT     │ ← Servo 13: Tilt (Y-axis)  │
│              │    SERVO        │   XL330 (lighter)          │
│              └────────┬────────┘                            │
│                       │                                     │
│              ┌────────┴────────┐                            │
│              │   NECK PAN      │ ← Servo 14: Pan (Z-axis)   │
│              │    SERVO        │   XL330 (lighter)          │
│              └────────┬────────┘                            │
│                       │                                     │
│                  [TO TORSO]                                 │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

#### Head Parts List

| Part | Dimensions | Print Time | Qty |
|------|------------|------------|-----|
| head_shell.stl | 60x60x50mm | 2.5 hrs | 1 |
| neck_bracket.stl | 40x40x30mm | 1 hr | 1 |
| camera_mount.stl | 30x30x20mm | 30 min | 1 |

---

## Complete Parts Summary

### 3D Printed Parts

| Category | Parts | Total Print Time | Filament |
|----------|-------|------------------|----------|
| Torso | 5 parts | ~12 hours | ~200g PETG |
| Arms (x2) | 10 parts | ~15 hours | ~150g PETG |
| Legs (x2) | 14 parts | ~22 hours | ~300g PETG |
| Head | 3 parts | ~4 hours | ~50g PETG |
| Feet (x2) | 4 parts | ~6 hours | ~100g TPU |
| **TOTAL** | **36 parts** | **~59 hours** | **~800g** |

### Hardware Required

| Item | Size | Qty | Notes |
|------|------|-----|-------|
| M2.5 x 6mm screw | - | 60 | Servo mounting |
| M2.5 x 10mm screw | - | 40 | Bracket assembly |
| M3 x 8mm screw | - | 30 | Frame assembly |
| M3 x 12mm screw | - | 20 | Through-frame |
| M2.5 heat-set insert | - | 100 | For PETG |
| M3 heat-set insert | - | 50 | For PETG |
| M2 x 5mm screw | - | 28 | Servo horns (14 servos x 2) |

---

## Servo Mounting Details

### XL430 Mounting Pattern

```
        ┌─────────────────────┐
        │    ○       ○        │  ← M2.5 mounting holes
        │                     │    (8 holes total)
        │  ┌───────────────┐  │
        │  │               │  │
        │  │    SERVO      │  │
        │  │    BODY       │  │
        │  │               │  │
        │  └───────┬───────┘  │
        │          │          │
        │    ○     │     ○    │
        │          │          │
        └──────────┼──────────┘
                   │
              ┌────┴────┐
              │  HORN   │  ← M2 screw pattern
              │  ○    ○ │    for attaching links
              │    ○    │
              │  ○    ○ │
              └─────────┘

  Mounting hole pattern:
  - 8x M2.5 threaded holes on body
  - Horn: 4x M2 + 1x center hole
```

### Bracket Design Guidelines

```
┌─────────────────────────────────────────────────────────────┐
│              SERVO BRACKET DESIGN RULES                     │
├─────────────────────────────────────────────────────────────┤
│                                                             │
│  1. Wall thickness: minimum 3mm around servo                │
│                                                             │
│  2. Clearance: 1mm around servo body for insertion          │
│                                                             │
│  3. Cable routing: 8mm diameter channel for Dynamixel cable │
│                                                             │
│  4. Horn clearance: Account for 12mm horn radius            │
│                                                             │
│  5. Heat-set inserts: Use M2.5 inserts, 4mm depth           │
│                                                             │
│       ┌─────────────────────────┐                           │
│       │     3mm wall            │                           │
│       │  ┌─────────────────┐    │                           │
│       │  │   1mm gap       │    │                           │
│       │  │  ┌───────────┐  │    │                           │
│       │  │  │           │  │    │                           │
│       │  │  │   SERVO   │  │    │                           │
│       │  │  │           │  │    │                           │
│       │  │  └───────────┘  │    │                           │
│       │  │                 │    │                           │
│       │  └─────────────────┘    │                           │
│       │  ┌──┐   cable   ┌──┐    │                           │
│       │  │  │  channel  │  │    │                           │
│       └──┴──┴───────────┴──┴────┘                           │
│                                                             │
└─────────────────────────────────────────────────────────────┘
```

---

## Print Settings

### PETG (Structural Parts)

| Setting | Value |
|---------|-------|
| Nozzle Temp | 240-250°C |
| Bed Temp | 70-80°C |
| Layer Height | 0.2mm |
| Wall Count | 4 |
| Infill | 40% (gyroid) |
| Speed | 50mm/s |
| Supports | Tree supports where needed |

### TPU (Feet)

| Setting | Value |
|---------|-------|
| Nozzle Temp | 220-230°C |
| Bed Temp | 50°C |
| Layer Height | 0.2mm |
| Wall Count | 3 |
| Infill | 30% |
| Speed | 25mm/s (slow!) |
| Retraction | Minimal (2mm) |

---

## Assembly Sequence

### Phase 1: Torso (Day 1)
1. Print torso main body
2. Install heat-set inserts
3. Mount shoulder servos (IDs 1, 4)
4. Install Teensy + wiring harness
5. Test servo communication

### Phase 2: Arms (Day 2-3)
1. Print arm parts
2. Assemble left arm:
   - Shoulder bracket + servo (ID 2)
   - Upper arm tube
   - Elbow bracket + servo (ID 3)
   - Forearm tube
   - Hand
3. Repeat for right arm (IDs 5, 6)
4. Attach arms to torso
5. Test arm movement

### Phase 3: Legs (Day 4-5)
1. Print leg parts
2. Assemble left leg:
   - Hip mount + servo (ID 7)
   - Hip bracket
   - Thigh tube
   - Knee bracket + servo (ID 8)
   - Shin tube
   - Ankle bracket + servo (ID 9)
   - Foot (TPU)
3. Repeat for right leg (IDs 10, 11, 12)
4. Attach legs to pelvis
5. Test leg movement

### Phase 4: Head & Final (Day 6)
1. Print head parts
2. Assemble neck + head (IDs 13, 14)
3. Attach to torso
4. Install Raspberry Pi
5. Install battery
6. Final wiring
7. Full system test

---

## CAD File Structure

```
/karate_robot/cad/
├── assemblies/
│   ├── sensei_full.step
│   ├── sensei_full.f3d (Fusion 360)
│   └── sensei_full.fcstd (FreeCAD)
├── parts/
│   ├── torso/
│   │   ├── torso_main.stl
│   │   ├── torso_back_cover.stl
│   │   ├── battery_door.stl
│   │   ├── shoulder_mount_L.stl
│   │   └── shoulder_mount_R.stl
│   ├── arm_left/
│   │   ├── shoulder_bracket_L.stl
│   │   ├── upper_arm_L.stl
│   │   ├── elbow_bracket_L.stl
│   │   ├── forearm_L.stl
│   │   └── hand_L.stl
│   ├── arm_right/
│   │   └── ... (mirrored)
│   ├── leg_left/
│   │   ├── hip_mount_L.stl
│   │   ├── hip_bracket_L.stl
│   │   ├── thigh_L.stl
│   │   ├── knee_bracket_L.stl
│   │   ├── shin_L.stl
│   │   ├── ankle_bracket_L.stl
│   │   └── foot_L.stl
│   ├── leg_right/
│   │   └── ... (mirrored)
│   └── head/
│       ├── head_shell.stl
│       ├── neck_bracket.stl
│       └── camera_mount.stl
└── drawings/
    ├── assembly_instructions.pdf
    └── wiring_diagram.pdf
```

---

## Next Steps

1. [ ] Create detailed CAD models in Fusion 360 / FreeCAD
2. [ ] Generate STL files for 3D printing
3. [ ] Print test bracket for servo fit check
4. [ ] Order hardware (screws, inserts)
5. [ ] Begin printing while waiting for servo delivery

---

## Resources

- [Dynamixel XL430 Drawings](https://emanual.robotis.com/docs/en/dxl/x/xl430-w250/)
- [Fusion 360 for Robotics](https://www.autodesk.com/products/fusion-360)
- [FreeCAD (Free alternative)](https://www.freecadweb.org/)
- [Prusa Slicer](https://www.prusa3d.com/prusaslicer/)

---

*Last updated: January 28, 2026*
