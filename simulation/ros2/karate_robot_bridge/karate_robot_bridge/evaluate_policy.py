#!/usr/bin/env python3
"""
Policy evaluation script for trained walking policies.
Runs multiple episodes and reports statistics.
"""

import argparse
import json
import time
from dataclasses import dataclass
from pathlib import Path
from typing import List, Optional

import numpy as np


@dataclass
class EpisodeResult:
    """Results from a single evaluation episode."""
    episode_id: int
    total_reward: float
    episode_length: int
    max_forward_distance: float
    fell: bool
    reached_goal: bool
    avg_velocity: float


@dataclass
class EvaluationReport:
    """Aggregated evaluation results."""
    num_episodes: int
    success_rate: float
    avg_reward: float
    std_reward: float
    avg_episode_length: float
    avg_distance: float
    fall_rate: float
    avg_velocity: float
    
    def to_dict(self) -> dict:
        return {
            "num_episodes": self.num_episodes,
            "success_rate": f"{self.success_rate:.1%}",
            "avg_reward": f"{self.avg_reward:.2f}",
            "std_reward": f"{self.std_reward:.2f}",
            "avg_episode_length": f"{self.avg_episode_length:.1f}",
            "avg_distance_m": f"{self.avg_distance:.3f}",
            "fall_rate": f"{self.fall_rate:.1%}",
            "avg_velocity_m_s": f"{self.avg_velocity:.3f}",
        }
    
    def print_report(self) -> None:
        print("\n" + "=" * 60)
        print("WALKING POLICY EVALUATION REPORT")
        print("=" * 60)
        print(f"Episodes evaluated: {self.num_episodes}")
        print(f"Success rate:       {self.success_rate:.1%}")
        print(f"Fall rate:          {self.fall_rate:.1%}")
        print(f"Avg reward:         {self.avg_reward:.2f} ± {self.std_reward:.2f}")
        print(f"Avg episode length: {self.avg_episode_length:.1f} steps")
        print(f"Avg distance:       {self.avg_distance:.3f} m")
        print(f"Avg velocity:       {self.avg_velocity:.3f} m/s")
        print("=" * 60)
        
        # Qualitative assessment
        print("\nQUALITATIVE ASSESSMENT:")
        if self.success_rate >= 0.8:
            print("✅ EXCELLENT: Policy successfully walks in most episodes")
        elif self.success_rate >= 0.5:
            print("🟡 GOOD: Policy walks but has room for improvement")
        elif self.success_rate >= 0.2:
            print("🟠 FAIR: Policy shows learning but needs more training")
        else:
            print("❌ POOR: Policy needs significant more training")
        
        if self.fall_rate <= 0.1:
            print("✅ Very stable - rarely falls")
        elif self.fall_rate <= 0.3:
            print("🟡 Moderately stable - occasional falls")
        else:
            print("🟠 Unstable - frequent falls")
        
        if self.avg_velocity >= 0.25:
            print("✅ Good walking speed")
        elif self.avg_velocity >= 0.1:
            print("🟡 Slow but moving forward")
        else:
            print("🟠 Minimal forward progress")
        print("")


def aggregate_results(episodes: List[EpisodeResult]) -> EvaluationReport:
    """Aggregate episode results into a report."""
    rewards = [e.total_reward for e in episodes]
    lengths = [e.episode_length for e in episodes]
    distances = [e.max_forward_distance for e in episodes]
    velocities = [e.avg_velocity for e in episodes]
    
    return EvaluationReport(
        num_episodes=len(episodes),
        success_rate=sum(1 for e in episodes if e.reached_goal) / len(episodes),
        avg_reward=np.mean(rewards),
        std_reward=np.std(rewards),
        avg_episode_length=np.mean(lengths),
        avg_distance=np.mean(distances),
        fall_rate=sum(1 for e in episodes if e.fell) / len(episodes),
        avg_velocity=np.mean(velocities),
    )


def evaluate_with_ros2(
    model_path: str,
    num_episodes: int = 20,
    max_steps_per_episode: int = 2000,
    render: bool = False,
) -> EvaluationReport:
    """
    Evaluate policy using ROS2 environment.
    
    This function should be called within a ROS2 context with the
    simulation running.
    """
    # Import ROS2 and ML dependencies
    import rclpy
    from stable_baselines3 import PPO
    
    print(f"Loading policy from {model_path}...")
    model = PPO.load(model_path)
    
    print(f"Running {num_episodes} evaluation episodes...")
    episodes: List[EpisodeResult] = []
    
    # Note: In actual implementation, this would interface with
    # the ROS2 training environment via topics
    # For now, this is a template showing the evaluation structure
    
    for ep in range(num_episodes):
        print(f"  Episode {ep + 1}/{num_episodes}...", end=" ", flush=True)
        
        # Placeholder for actual ROS2 evaluation
        # In practice, this would:
        # 1. Reset the simulation
        # 2. Run policy inference
        # 3. Collect metrics
        
        # Simulated result for template
        result = EpisodeResult(
            episode_id=ep,
            total_reward=0.0,  # Would come from actual evaluation
            episode_length=0,
            max_forward_distance=0.0,
            fell=False,
            reached_goal=False,
            avg_velocity=0.0,
        )
        episodes.append(result)
        print("done")
    
    return aggregate_results(episodes)


def main():
    parser = argparse.ArgumentParser(description="Evaluate trained walking policy")
    parser.add_argument(
        "--model", 
        type=str, 
        required=True,
        help="Path to trained model (.zip file)"
    )
    parser.add_argument(
        "--episodes", 
        type=int, 
        default=20,
        help="Number of evaluation episodes (default: 20)"
    )
    parser.add_argument(
        "--output", 
        type=str, 
        default="",
        help="Output JSON file for results"
    )
    parser.add_argument(
        "--render", 
        action="store_true",
        help="Render simulation (requires display)"
    )
    args = parser.parse_args()
    
    # Run evaluation
    report = evaluate_with_ros2(
        model_path=args.model,
        num_episodes=args.episodes,
        render=args.render,
    )
    
    # Print report
    report.print_report()
    
    # Save to file if requested
    if args.output:
        output_path = Path(args.output)
        output_path.write_text(json.dumps(report.to_dict(), indent=2))
        print(f"Results saved to {output_path}")


if __name__ == "__main__":
    main()
