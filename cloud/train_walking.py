#!/usr/bin/env python3
"""
Standalone walking training script for GCP.
Uses a simplified bipedal walking environment to train the core locomotion policy.
This can be fine-tuned on the full Gazebo simulation later.
"""

import os
import sys
import argparse
from datetime import datetime

# Training parameters
DEFAULT_TIMESTEPS = 10_000_000
CHECKPOINT_FREQ = 100_000

def main():
    parser = argparse.ArgumentParser(description="Train walking policy")
    parser.add_argument("--timesteps", type=int, default=DEFAULT_TIMESTEPS,
                        help="Total training timesteps")
    parser.add_argument("--resume", type=str, default="",
                        help="Path to checkpoint to resume from")
    parser.add_argument("--output", type=str, default="/tmp/karate_robot_walk",
                        help="Output directory for models and logs")
    args = parser.parse_args()

    # Import after argparse to show help faster
    import gymnasium as gym
    from stable_baselines3 import PPO
    from stable_baselines3.common.callbacks import CheckpointCallback
    from stable_baselines3.common.vec_env import DummyVecEnv, VecNormalize

    # Create output directory
    os.makedirs(args.output, exist_ok=True)
    
    print("=" * 60)
    print("KARATE ROBOT WALKING TRAINING")
    print("=" * 60)
    print(f"Total timesteps: {args.timesteps:,}")
    print(f"Checkpoint every: {CHECKPOINT_FREQ:,} steps")
    print(f"Output directory: {args.output}")
    print(f"Started at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    print("=" * 60)

    # Use BipedalWalker - a 2D bipedal robot that learns to walk
    # This is a good proxy for basic walking locomotion
    # The policy architecture transfers well to more complex humanoid tasks
    print("\nCreating BipedalWalker-v3 environment...")
    env = gym.make("BipedalWalker-v3")
    env = DummyVecEnv([lambda: env])
    env = VecNormalize(env, norm_obs=True, norm_reward=True, clip_obs=10.0)

    # Setup checkpoint callback
    checkpoint_callback = CheckpointCallback(
        save_freq=CHECKPOINT_FREQ,
        save_path=args.output,
        name_prefix="walk_checkpoint",
        save_replay_buffer=False,
        save_vecnormalize=True,
    )

    # Create or load model
    if args.resume and os.path.exists(args.resume):
        print(f"\nResuming from checkpoint: {args.resume}")
        model = PPO.load(args.resume, env=env, tensorboard_log=args.output)
    else:
        print("\nCreating new PPO model...")
        model = PPO(
            "MlpPolicy",
            env,
            learning_rate=3e-4,
            n_steps=2048,
            batch_size=64,
            n_epochs=10,
            gamma=0.99,
            gae_lambda=0.95,
            clip_range=0.2,
            ent_coef=0.01,
            verbose=1,
            tensorboard_log=args.output,
            device="auto",  # Will use GPU if available
        )

    print(f"\nStarting training for {args.timesteps:,} timesteps...")
    print("Progress will be shown below:\n")
    
    model.learn(
        total_timesteps=args.timesteps,
        callback=checkpoint_callback,
        progress_bar=True,
    )

    # Save final model
    final_path = os.path.join(args.output, "walking_policy_final")
    model.save(final_path)
    env.save(os.path.join(args.output, "vec_normalize.pkl"))
    
    print("\n" + "=" * 60)
    print("TRAINING COMPLETE")
    print("=" * 60)
    print(f"Final model saved to: {final_path}.zip")
    print(f"Finished at: {datetime.now().strftime('%Y-%m-%d %H:%M:%S')}")
    
    # Upload to GCS bucket
    bucket = os.environ.get("GCS_BUCKET", "robottraining-485701-training")
    print(f"\nUploading results to gs://{bucket}/...")
    os.system(f"gcloud storage cp -r {args.output}/* gs://{bucket}/walking/")
    print("Upload complete!")

if __name__ == "__main__":
    main()
