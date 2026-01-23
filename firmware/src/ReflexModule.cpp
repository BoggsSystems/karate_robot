// ReflexModule.cpp
// Bushido note: Kime (focus) is honored, but Gi (justice) forbids harm.

#include "HardwareBridge.h"

namespace karate_robot {

namespace {

// Trip point for collision detection based on current spike (fixed-point 14-bit).
constexpr int16_t kCurrentSpikeTrip = static_cast<int16_t>(6500);

}  // namespace

void processReflex(RobotRegisterMap& regs) {
  for (int joint = 0; joint < 7; ++joint) {
    JointRegisters& j = regs.joints[joint];

    if (regs.safety_halt_latch != 0) {
      // Bushido: when safety is invoked, stillness is the highest discipline.
      j.output_angle = clampFixed14(j.actual_angle);
      continue;
    }

    const bool current_spike = j.current_spike > kCurrentSpikeTrip;
    if (current_spike) {
      // A collision is sensed; latch safety to protect the Sensei and the robot.
      regs.safety_halt_latch = 1;
      j.output_angle = clampFixed14(j.actual_angle);
      continue;
    }

    const bool over_torque = j.actual_torque > j.kime_intensity;
    if (over_torque) {
      // Bushido: excessive force is tempered; clamp output to the present state.
      j.output_angle = clampFixed14(j.actual_angle);
    } else {
      // No injustice detected; allow the Brain's intent to proceed.
      j.output_angle = clampFixed14(j.target_angle);
    }
  }
}

}  // namespace karate_robot
