// HardwareBridge.h
// Contract between the Python Brain and the C++ Nervous System.
// Bushido note: the Brain may "dream" movement, but the Reflexes enforce Gi (justice).

#pragma once

#include <cstdint>

namespace karate_robot {

// 14-bit signed fixed-point range: [-8192, 8191]
constexpr int16_t kFixed14Min = static_cast<int16_t>(-8192);
constexpr int16_t kFixed14Max = static_cast<int16_t>(8191);

inline int16_t clampFixed14(int16_t value) {
  if (value < kFixed14Min) {
    return kFixed14Min;
  }
  if (value > kFixed14Max) {
    return kFixed14Max;
  }
  return value;
}

struct JointRegisters {
  // Desired joint angle from the Brain (fixed-point 14-bit).
  int16_t target_angle = 0;
  // Measured joint angle from encoders (fixed-point 14-bit).
  int16_t actual_angle = 0;
  // Measured joint torque (fixed-point 14-bit).
  int16_t actual_torque = 0;
  // Justice threshold: if torque exceeds this, Reflexes clamp motion.
  int16_t kime_intensity = 0;
  // Reflex output command after safety logic (fixed-point 14-bit).
  int16_t output_angle = 0;
  // High current spike marker from power stage (fixed-point 14-bit).
  int16_t current_spike = 0;
};

struct RobotRegisterMap {
  // Seven joints for a 7-DOF humanoid arm.
  JointRegisters joints[7];
  // Global safety latch: once tripped, movement halts until reset.
  uint8_t safety_halt_latch = 0;
};

}  // namespace karate_robot
