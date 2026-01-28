# Karate Robot Motion Training Plan

## Overview

This document outlines the complete training requirements for teaching the karate robot all necessary motions, from basic locomotion to advanced kata sequences.

**Training Method:** Proximal Policy Optimization (PPO) via Stable-Baselines3  
**Simulation:** Gazebo + Custom ROS 2 Environment  
**Estimated Total:** ~689M timesteps, ~98.5 GPU hours

---

## Motion Inventory & Training Estimates

### Tier 1: Foundation (Must Have First)

| Motion | Complexity | Est. Steps | GPU Hours | Notes |
|--------|-----------|------------|-----------|-------|
| **Standing Balance** | Low | 1M | 0.15 | Prerequisite for everything |
| **Walking Forward** | High | 10M | 1.4 | Current training |
| **Walking Backward** | Medium | 5M | 0.7 | Transfer from forward |
| **Turning (pivot)** | Medium | 5M | 0.7 | |
| **Stable Stance (ready)** | Low | 2M | 0.3 | Base position |

**Tier 1 Subtotal: 23M steps, ~3.3 GPU hours**

---

### Tier 2: Stances (Karate Fundamentals)

| Motion | Complexity | Est. Steps | GPU Hours | Notes |
|--------|-----------|------------|-----------|-------|
| **Front Stance** (zenkutsu-dachi) | Medium | 3M | 0.4 | Deep forward stance |
| **Back Stance** (kokutsu-dachi) | Medium | 3M | 0.4 | Weight on rear leg |
| **Horse Stance** (kiba-dachi) | Medium | 3M | 0.4 | Wide squat |
| **Cat Stance** (neko-ashi-dachi) | High | 5M | 0.7 | Balance on one leg |
| **Stance Transitions** | High | 10M | 1.4 | Moving between stances |

**Tier 2 Subtotal: 24M steps, ~3.4 GPU hours**

---

### Tier 3: Punches & Strikes

| Motion | Complexity | Est. Steps | GPU Hours | Notes |
|--------|-----------|------------|-----------|-------|
| **Straight Punch** (choku-zuki) | Low | 2M | 0.3 | Basic arm extension |
| **Reverse Punch** (gyaku-zuki) | Medium | 4M | 0.6 | Hip rotation involved |
| **Jab** | Low | 2M | 0.3 | Quick, no hip |
| **Uppercut** | Medium | 4M | 0.6 | |
| **Hammer Fist** | Low | 2M | 0.3 | |
| **Backfist** | Medium | 3M | 0.4 | |
| **Elbow Strike** | Medium | 3M | 0.4 | |
| **Punch Combos** (1-2, 1-2-3) | High | 10M | 1.4 | Sequencing |

**Tier 3 Subtotal: 30M steps, ~4.3 GPU hours**

---

### Tier 4: Kicks (Most Complex)

| Motion | Complexity | Est. Steps | GPU Hours | Notes |
|--------|-----------|------------|-----------|-------|
| **Front Kick** (mae-geri) | High | 15M | 2.1 | Balance + power |
| **Roundhouse Kick** (mawashi-geri) | Very High | 20M | 2.9 | Rotation + balance |
| **Side Kick** (yoko-geri) | Very High | 20M | 2.9 | Chamber + thrust |
| **Back Kick** (ushiro-geri) | Very High | 25M | 3.6 | Can't see target |
| **Crescent Kick** | High | 15M | 2.1 | Arc motion |
| **Knee Strike** | Medium | 5M | 0.7 | |
| **Kick Combos** | Very High | 30M | 4.3 | Multi-kick sequences |

**Tier 4 Subtotal: 130M steps, ~18.6 GPU hours**

---

### Tier 5: Blocks & Defense

| Motion | Complexity | Est. Steps | GPU Hours | Notes |
|--------|-----------|------------|-----------|-------|
| **High Block** (jodan-uke) | Low | 2M | 0.3 | |
| **Middle Block** (chudan-uke) | Low | 2M | 0.3 | |
| **Low Block** (gedan-barai) | Low | 2M | 0.3 | |
| **Knife Hand Block** (shuto-uke) | Medium | 3M | 0.4 | |
| **X-Block** | Medium | 3M | 0.4 | |
| **Dodge/Slip** | High | 10M | 1.4 | Weight transfer |
| **Block + Counter** | High | 15M | 2.1 | Reaction timing |

**Tier 5 Subtotal: 37M steps, ~5.3 GPU hours**

---

### Tier 6: Advanced Combinations

| Motion | Complexity | Est. Steps | GPU Hours | Notes |
|--------|-----------|------------|-----------|-------|
| **Block → Punch** | High | 10M | 1.4 | |
| **Block → Kick** | Very High | 15M | 2.1 | |
| **Kick → Punch Combo** | Very High | 20M | 2.9 | |
| **Full Combo Sequences** | Extreme | 30M | 4.3 | 4+ move chains |

**Tier 6 Subtotal: 75M steps, ~10.7 GPU hours**

---

### Tier 7: Kata & Sparring (Future)

| Motion | Complexity | Est. Steps | GPU Hours | Notes |
|--------|-----------|------------|-----------|-------|
| **Basic Kata** (Taikyoku) | Extreme | 50M | 7.1 | 20+ move sequence |
| **Intermediate Kata** | Extreme | 100M | 14.3 | |
| **Sparring (reactive)** | Extreme | 200M | 28.6 | Opponent modeling |
| **Push Recovery** | High | 20M | 2.9 | Robustness |

**Tier 7 Subtotal: 370M steps, ~52.9 GPU hours**

---

## Total Training Requirements

| Tier | Steps | GPU Hours | Cumulative |
|------|-------|-----------|------------|
| T1: Foundation | 23M | 3.3 | 3.3 hrs |
| T2: Stances | 24M | 3.4 | 6.7 hrs |
| T3: Punches | 30M | 4.3 | 11.0 hrs |
| T4: Kicks | 130M | 18.6 | 29.6 hrs |
| T5: Blocks | 37M | 5.3 | 34.9 hrs |
| T6: Combos | 75M | 10.7 | 45.6 hrs |
| T7: Kata/Sparring | 370M | 52.9 | 98.5 hrs |

**Grand Total: ~689M steps, ~98.5 GPU hours**

---

## Azure Cost Projections

### By VM Type

| VM Type | GPU | Speed | Time for All | Cost |
|---------|-----|-------|--------------|------|
| NC6s_v3 (Spot) | 1x V100 | 2K fps | 98.5 hrs | **~$50** |
| NC6s_v3 (Regular) | 1x V100 | 2K fps | 98.5 hrs | ~$295 |
| NC12s_v3 (Spot) | 2x V100 | 4K fps | 49 hrs | ~$50 |
| NC24s_v3 (Spot) | 4x V100 | 8K fps | 25 hrs | ~$50 |

### By Development Phase

| Phase | Motions | Steps | Spot Cost | Timeline |
|-------|---------|-------|-----------|----------|
| **MVP** (Walking + Basic) | T1 + basic kicks | 50M | ~$4 | Week 1-2 |
| **Alpha** (Core Karate) | T1-T4 | 207M | ~$15 | Month 1 |
| **Beta** (Full Repertoire) | T1-T6 | 319M | ~$23 | Month 2-3 |
| **Production** (All) | T1-T7 | 689M | ~$50 | Month 4-6 |

---

## Recommended Training Strategy

```
┌─────────────────────────────────────────────────────────────┐
│                 TRAINING PIPELINE                           │
│                                                             │
│  Phase 1: Foundation (Local Mac)                            │
│  └── Walking 10M steps (current - 35 hrs)                   │
│                                                             │
│  Phase 2: Core Skills (Azure Spot)                          │
│  ├── Parallel training: 4 VMs                               │
│  ├── VM1: Stances (24M)                                     │
│  ├── VM2: Punches (30M)                                     │
│  ├── VM3: Basic Kicks (50M)                                 │
│  └── VM4: Blocks (37M)                                      │
│  └── Total: ~6 hours, ~$12                                  │
│                                                             │
│  Phase 3: Advanced (Azure Spot)                             │
│  ├── Transfer learning from Phase 2                         │
│  ├── Advanced kicks (80M)                                   │
│  └── Combinations (75M)                                     │
│  └── Total: ~11 hours, ~$22                                 │
│                                                             │
│  Phase 4: Integration                                       │
│  └── Kata, sparring (370M)                                  │
│  └── Total: ~26 hours, ~$50 (parallel)                      │
└─────────────────────────────────────────────────────────────┘
```

---

## Budget Summary

| Scenario | Total Cost | Timeline |
|----------|------------|----------|
| **Minimum Viable** (walks + kicks) | ~$10 | 2 weeks |
| **Full Karate Basics** | ~$50 | 1-2 months |
| **Competition Ready** | ~$100-150 | 3-6 months |

---

## Proficiency Levels Reference

| Level | Timesteps | Expected Behavior |
|-------|-----------|-------------------|
| **1. Flailing** | 0 - 200K | Random movements, falls immediately |
| **2. Balance Attempt** | 200K - 500K | Tries to stay upright, still falls quickly |
| **3. Stumbling** | 500K - 1M | Can stand briefly, takes 1-2 steps before falling |
| **4. Unstable Walk** | 1M - 3M | Walks a few meters, wobbly |
| **5. Basic Walk** | 3M - 5M | Walks forward reliably, simple gait |
| **6. Stable Walk** | 5M - 10M | Consistent gait, recovers from small bumps |
| **7. Natural Walk** | 10M - 50M | Smooth motion, efficient energy use |
| **8. Robust Walk** | 50M+ | Handles terrain, pushes, obstacles |

---

## Technical Notes

### Algorithm
- **PPO (Proximal Policy Optimization)** - On-policy, sample efficient for locomotion
- Alternative: **SAC** for sample efficiency if compute limited

### Reward Shaping
Each motion needs custom reward functions:
- **Walking**: forward velocity + upright bonus + gait symmetry
- **Kicks**: foot velocity at contact + balance + return to stance
- **Punches**: hand velocity + hip rotation + stability
- **Blocks**: arm position accuracy + timing + stance maintenance

### Transfer Learning
Later motions can initialize from earlier policies:
- Kicks initialize from stable stance
- Combos initialize from individual moves
- Reduces training time by 30-50%

---

## Files & Resources

- Training environment: `simulation/ros2/karate_robot_bridge/karate_robot_bridge/training_env_node.py`
- RL trainer: `simulation/ros2/karate_robot_bridge/karate_robot_bridge/rl_trainer_node.py`
- Reward profiles: `simulation/ros2/karate_robot_bridge/config/walk_reward_profiles.yaml`
- Checkpoints: `~/karate_robot/training_output/`
- Models: `~/karate_robot/models/`

---

*Last updated: January 28, 2026*
