#!/bin/bash
# Comprehensive evaluation script for trained walking policy
# Run this after training completes

set -e

MODEL_DIR="${HOME}/karate_robot/models"
TRAINING_OUTPUT="${HOME}/karate_robot/training_output_v2"
GCP_VM="karate-training-docker"
GCP_ZONE="us-central1-a"

echo "=============================================="
echo "  KARATE ROBOT WALKING POLICY EVALUATION"
echo "=============================================="
echo ""

# Step 1: Download model from GCP
echo "Step 1: Downloading trained model from GCP..."
mkdir -p "$MODEL_DIR"

# Download checkpoint files
gcloud compute scp "${GCP_VM}:~/training_output_v2/*.zip" "$MODEL_DIR/" --zone="$GCP_ZONE" 2>/dev/null || true

# Download TensorBoard logs
mkdir -p "$MODEL_DIR/tensorboard"
gcloud compute scp -r "${GCP_VM}:~/training_output_v2/PPO_1" "$MODEL_DIR/tensorboard/" --zone="$GCP_ZONE" 2>/dev/null || true

echo "  Downloaded to: $MODEL_DIR"
echo ""

# Step 2: List downloaded files
echo "Step 2: Downloaded files:"
ls -lh "$MODEL_DIR"/*.zip 2>/dev/null || echo "  No .zip files found"
echo ""

# Step 3: Check training metrics
echo "Step 3: Training Summary"
if [ -d "$MODEL_DIR/tensorboard/PPO_1" ]; then
    echo "  TensorBoard logs available at: $MODEL_DIR/tensorboard/PPO_1"
    echo "  Run: tensorboard --logdir $MODEL_DIR/tensorboard"
else
    echo "  TensorBoard logs not found"
fi
echo ""

# Step 4: Quick model info
echo "Step 4: Model Information"
LATEST_MODEL=$(ls -t "$MODEL_DIR"/*.zip 2>/dev/null | head -1)
if [ -n "$LATEST_MODEL" ]; then
    echo "  Latest model: $LATEST_MODEL"
    echo "  Size: $(du -h "$LATEST_MODEL" | cut -f1)"
    
    # Extract checkpoint step from filename if present
    if [[ "$LATEST_MODEL" =~ ([0-9]+)_steps ]]; then
        echo "  Training steps: ${BASH_REMATCH[1]}"
    fi
else
    echo "  No model found!"
fi
echo ""

# Step 5: Evaluation instructions
echo "Step 5: To run simulation evaluation:"
echo ""
echo "  # Option A: Run in Docker locally"
echo "  cd ~/karate_robot"
echo "  docker run -it --rm \\"
echo "    -v $MODEL_DIR:/models \\"
echo "    karate-robot:v2 \\"
echo "    ros2 launch karate_robot_sim sensei_evaluate.launch.py \\"
echo "      model_path:=/models/$(basename "$LATEST_MODEL")"
echo ""
echo "  # Option B: Run on GCP VM"
echo "  gcloud compute ssh $GCP_VM --zone=$GCP_ZONE --command=\"\\"
echo "    sudo docker exec karate_train_v2 \\"
echo "    ros2 launch karate_robot_sim sensei_evaluate.launch.py\""
echo ""

# Step 6: What to look for
echo "=============================================="
echo "  EVALUATION CRITERIA"
echo "=============================================="
echo ""
echo "  GOOD POLICY indicators:"
echo "    ✅ Episode reward > -100 (was -660 at start)"
echo "    ✅ Episode length > 1500 steps"
echo "    ✅ Forward velocity > 0.2 m/s"
echo "    ✅ Fall rate < 20%"
echo ""
echo "  NEEDS MORE TRAINING if:"
echo "    ❌ Episode reward < -300"
echo "    ❌ Falls immediately (length < 200)"
echo "    ❌ No forward progress"
echo ""

# Step 7: Stop GCP VM reminder
echo "=============================================="
echo "  IMPORTANT: STOP GCP VM TO AVOID CHARGES"
echo "=============================================="
echo ""
echo "  gcloud compute instances stop $GCP_VM --zone=$GCP_ZONE"
echo ""
