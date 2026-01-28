#!/bin/bash
# Create a GPU VM for training
# Cost: ~$0.35/hr for T4 GPU spot instance

set -e

PROJECT_ID="robottraining-485701"
ZONE="us-central1-a"
VM_NAME="karate-training"
MACHINE_TYPE="n1-standard-4"  # 4 vCPUs, 15GB RAM
GPU_TYPE="nvidia-tesla-t4"

# Use spot/preemptible for 60-70% cost savings
SPOT_FLAG="--provisioning-model=SPOT"

echo "=== Creating Training VM ==="
echo "Machine: $MACHINE_TYPE with $GPU_TYPE"
echo "Zone: $ZONE"
echo "Using Spot pricing for cost savings"
echo ""

gcloud compute instances create $VM_NAME \
    --project=$PROJECT_ID \
    --zone=$ZONE \
    --machine-type=$MACHINE_TYPE \
    --accelerator=type=$GPU_TYPE,count=1 \
    --image-family=pytorch-latest-gpu \
    --image-project=deeplearning-platform-release \
    --boot-disk-size=100GB \
    --boot-disk-type=pd-ssd \
    --maintenance-policy=TERMINATE \
    --tags=tensorboard \
    $SPOT_FLAG \
    --metadata=install-nvidia-driver=True

echo ""
echo "=== VM Created ==="
echo "To connect: gcloud compute ssh $VM_NAME --zone=$ZONE"
echo "To stop:    gcloud compute instances stop $VM_NAME --zone=$ZONE"
echo "To delete:  gcloud compute instances delete $VM_NAME --zone=$ZONE"
