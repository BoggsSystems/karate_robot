#!/bin/bash
# Entrypoint for GPU training container

set -e

# Source ROS 2
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# Default values
TIMESTEPS=${TIMESTEPS:-10000000}
CHECKPOINT_FREQ=${CHECKPOINT_FREQ:-100000}
RESUME_PATH=${RESUME_PATH:-""}
OUTPUT_DIR=${OUTPUT_DIR:-/output}

mkdir -p $OUTPUT_DIR

echo "=== Karate Robot GPU Training ==="
echo "Timesteps: $TIMESTEPS"
echo "Checkpoint frequency: $CHECKPOINT_FREQ"
echo "Output: $OUTPUT_DIR"
if [ -n "$RESUME_PATH" ]; then
    echo "Resuming from: $RESUME_PATH"
fi
echo ""

# Run training
ros2 launch karate_robot_sim sensei_walk_train_long.launch.py \
    timesteps:=$TIMESTEPS \
    resume:=$RESUME_PATH \
    headless:=true

# Upload results to GCS if bucket is specified
if [ -n "$GCS_BUCKET" ]; then
    echo "Uploading results to $GCS_BUCKET..."
    gsutil -m cp -r $OUTPUT_DIR/* gs://$GCS_BUCKET/
fi

echo "=== Training Complete ==="
