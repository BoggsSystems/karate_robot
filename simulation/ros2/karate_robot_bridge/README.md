# Karate Robot Bridge

This package hosts ROS 2 nodes that connect the strategist, safety layer, and
training pipeline to the simulator.

## Training Interface (Phase 1)

The training pipeline uses the following topics:

- `/training/observation` (`std_msgs/Float64MultiArray`)
  - Layout: joint positions (ordered by `joint_names`), safety latch, state id
- `/training/action` (`std_msgs/Float64MultiArray`)
  - Layout: absolute joint targets in radians, ordered by `joint_names`
- `/training/reward` (`std_msgs/Float32`)
- `/training/done` (`std_msgs/Bool`)
- `/training/episode` (`std_msgs/Int32`)
- `/training/schema` (`std_msgs/String`)
  - JSON snapshot of the schema for trainer discovery

The schema is defined in `config/training_schema.yaml`.

## RL Trainer (PPO/SAC)

`rl_trainer` is a simple ROS-aware trainer that wraps Stable-Baselines3.
It subscribes to the training topics and learns a policy in-process.

Install dependencies:

```
pip install stable-baselines3 gymnasium
```

Run PPO training (default):

```
ros2 run karate_robot_bridge rl_trainer
```

Run SAC with custom timesteps:

```
ros2 run karate_robot_bridge rl_trainer --ros-args -p algo:=SAC -p total_timesteps:=50000
```
