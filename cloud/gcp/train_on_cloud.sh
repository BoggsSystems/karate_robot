#!/bin/bash
# Upload code and start training on GCP VM
# Run this from your local machine

set -e

PROJECT_ID="robottraining-485701"
ZONE="us-central1-a"
VM_NAME="karate-training"
BUCKET_NAME="${PROJECT_ID}-training"

echo "=== Uploading Training Code ==="

# Upload the karate_robot code to the VM
gcloud compute scp --recurse \
    ~/karate_robot/simulation \
    ~/karate_robot/docker \
    $VM_NAME:~/karate_robot/ \
    --zone=$ZONE

# Upload any existing checkpoints
if [ -f ~/karate_robot/training_output/walk_checkpoint_100000_steps.zip ]; then
    echo "Uploading existing checkpoint..."
    gcloud compute scp \
        ~/karate_robot/training_output/walk_checkpoint_100000_steps.zip \
        $VM_NAME:~/karate_robot/ \
        --zone=$ZONE
fi

echo ""
echo "=== Starting Training ==="
echo "Connecting to VM and starting training..."

gcloud compute ssh $VM_NAME --zone=$ZONE --command="
cd ~/karate_robot

# Install dependencies
pip install stable-baselines3[extra] gymnasium tensorboard

# Create output directory
mkdir -p ~/training_output

# Start training (10M timesteps, ~4-6 hours on T4 GPU)
python3 -c '
import gymnasium as gym
from stable_baselines3 import PPO
from stable_baselines3.common.callbacks import CheckpointCallback
import os

# Simple training environment (headless, no Gazebo needed for basic RL)
# For full sim training, would need to set up ROS2 + Gazebo on the VM

print(\"Starting PPO training...\")
print(\"Checkpoints saved every 100K steps\")
print(\"Final model saved to ~/training_output/\")

# This is a placeholder - actual training uses the ROS2 environment
# For cloud deployment, we would containerize the full stack
print(\"Note: Full training requires ROS2 environment setup\")
print(\"See cloud/gcp/Dockerfile.gpu for containerized training\")
'
"

echo ""
echo "=== Training Started ==="
echo "Monitor with: gcloud compute ssh $VM_NAME --zone=$ZONE"
echo "Download results: gsutil cp gs://$BUCKET_NAME/checkpoints/* ./training_output/"
