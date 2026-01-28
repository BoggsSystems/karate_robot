#!/usr/bin/env python3
"""
Domain randomization utilities for sim-to-real transfer.

These functions add realistic variations to training to help policies
generalize to real hardware.
"""

import random
import math
from dataclasses import dataclass
from typing import List, Dict, Optional


@dataclass
class RandomizationConfig:
    """Configuration for domain randomization."""
    
    # Observation noise
    joint_position_noise_std: float = 0.02  # radians (~1 degree)
    joint_velocity_noise_std: float = 0.1   # rad/s
    imu_orientation_noise_std: float = 0.01  # radians
    imu_angular_vel_noise_std: float = 0.05  # rad/s
    
    # Actuator modeling
    motor_delay_ms_min: float = 10.0   # minimum motor response delay
    motor_delay_ms_max: float = 50.0   # maximum motor response delay
    motor_strength_min: float = 0.85   # motor can be 85-115% of nominal
    motor_strength_max: float = 1.15
    
    # Physics randomization (applied at episode start)
    mass_scale_min: float = 0.9   # total mass variation ±10%
    mass_scale_max: float = 1.1
    friction_min: float = 0.5     # ground friction variation
    friction_max: float = 1.2
    
    # Action noise (simulates imperfect motor control)
    action_noise_std: float = 0.02  # radians
    
    # External disturbances
    random_push_probability: float = 0.01  # 1% chance per step
    push_force_max: float = 20.0  # Newtons
    
    # Latency variation
    observation_delay_steps_max: int = 2  # up to 2 steps of obs delay
    
    enabled: bool = True


class DomainRandomizer:
    """Applies domain randomization to observations and actions."""
    
    def __init__(self, config: Optional[RandomizationConfig] = None):
        self.config = config or RandomizationConfig()
        self.observation_buffer: List[List[float]] = []
        self.current_motor_strength: float = 1.0
        self.current_delay_steps: int = 0
        
    def reset_episode(self) -> Dict[str, float]:
        """Reset randomization for new episode. Returns episode parameters."""
        if not self.config.enabled:
            return {}
            
        # Randomize per-episode parameters
        self.current_motor_strength = random.uniform(
            self.config.motor_strength_min,
            self.config.motor_strength_max
        )
        self.current_delay_steps = random.randint(
            0, self.config.observation_delay_steps_max
        )
        
        # Clear observation buffer
        self.observation_buffer.clear()
        
        return {
            "motor_strength": self.current_motor_strength,
            "observation_delay_steps": self.current_delay_steps,
            "mass_scale": random.uniform(
                self.config.mass_scale_min,
                self.config.mass_scale_max
            ),
            "friction": random.uniform(
                self.config.friction_min,
                self.config.friction_max
            ),
        }
    
    def randomize_observation(
        self, 
        observation: List[float],
        joint_count: int = 14
    ) -> List[float]:
        """Add noise to observation to simulate real sensor noise."""
        if not self.config.enabled:
            return observation
            
        obs = list(observation)
        
        # Add noise to joint positions (first joint_count elements)
        for i in range(min(joint_count, len(obs))):
            obs[i] += random.gauss(0, self.config.joint_position_noise_std)
        
        # Add noise to IMU data (base pose: height, roll, pitch, yaw)
        # Assuming layout: [joints..., safety, state, height, roll, pitch, yaw, ...]
        imu_start = joint_count + 2  # after joints, safety_latch, state_id
        if len(obs) > imu_start + 3:
            # Roll, pitch, yaw noise
            for i in range(imu_start + 1, min(imu_start + 4, len(obs))):
                obs[i] += random.gauss(0, self.config.imu_orientation_noise_std)
        
        # Add noise to velocity readings if present
        vel_start = imu_start + 4  # after pose
        if len(obs) > vel_start + 2:
            for i in range(vel_start, min(vel_start + 3, len(obs))):
                obs[i] += random.gauss(0, self.config.imu_angular_vel_noise_std)
        
        # Apply observation delay
        self.observation_buffer.append(obs)
        if len(self.observation_buffer) > self.current_delay_steps + 1:
            self.observation_buffer.pop(0)
        
        if self.current_delay_steps > 0 and len(self.observation_buffer) > self.current_delay_steps:
            return self.observation_buffer[0]
        
        return obs
    
    def randomize_action(self, action: List[float]) -> List[float]:
        """Add noise to action to simulate imperfect motor control."""
        if not self.config.enabled:
            return action
            
        noisy_action = []
        for a in action:
            # Add action noise
            noisy = a + random.gauss(0, self.config.action_noise_std)
            # Apply motor strength variation
            noisy *= self.current_motor_strength
            noisy_action.append(noisy)
        
        return noisy_action
    
    def should_apply_push(self) -> bool:
        """Check if random external push should be applied this step."""
        if not self.config.enabled:
            return False
        return random.random() < self.config.random_push_probability
    
    def get_random_push(self) -> Dict[str, float]:
        """Generate random push force vector."""
        angle = random.uniform(0, 2 * math.pi)
        magnitude = random.uniform(0, self.config.push_force_max)
        return {
            "force_x": magnitude * math.cos(angle),
            "force_y": magnitude * math.sin(angle),
            "force_z": 0.0,
        }


# Convenience function for quick integration
def create_default_randomizer(enabled: bool = True) -> DomainRandomizer:
    """Create a domain randomizer with sensible defaults."""
    config = RandomizationConfig(enabled=enabled)
    return DomainRandomizer(config)


def create_conservative_randomizer() -> DomainRandomizer:
    """Create a more conservative randomizer for initial training."""
    config = RandomizationConfig(
        joint_position_noise_std=0.01,
        joint_velocity_noise_std=0.05,
        imu_orientation_noise_std=0.005,
        motor_strength_min=0.95,
        motor_strength_max=1.05,
        mass_scale_min=0.95,
        mass_scale_max=1.05,
        action_noise_std=0.01,
        random_push_probability=0.005,
        enabled=True,
    )
    return DomainRandomizer(config)


def create_aggressive_randomizer() -> DomainRandomizer:
    """Create aggressive randomizer for robust policies."""
    config = RandomizationConfig(
        joint_position_noise_std=0.05,
        joint_velocity_noise_std=0.2,
        imu_orientation_noise_std=0.02,
        motor_strength_min=0.8,
        motor_strength_max=1.2,
        mass_scale_min=0.85,
        mass_scale_max=1.15,
        friction_min=0.4,
        friction_max=1.5,
        action_noise_std=0.03,
        random_push_probability=0.02,
        push_force_max=30.0,
        enabled=True,
    )
    return DomainRandomizer(config)
