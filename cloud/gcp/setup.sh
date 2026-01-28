#!/bin/bash
# GCP Setup Script for Karate Robot Training
# Run this once to set up your cloud infrastructure

set -e

PROJECT_ID="robottraining-485701"
REGION="us-central1"
ZONE="us-central1-a"
BUCKET_NAME="${PROJECT_ID}-training"

echo "=== Setting up GCP for Robot Training ==="

# Set project
gcloud config set project $PROJECT_ID

# Enable required APIs
echo "Enabling APIs..."
gcloud services enable compute.googleapis.com
gcloud services enable storage.googleapis.com
gcloud services enable logging.googleapis.com

# Create storage bucket for models and checkpoints
echo "Creating storage bucket..."
gsutil mb -l $REGION gs://$BUCKET_NAME || echo "Bucket may already exist"

# Create firewall rule for TensorBoard (optional)
echo "Creating firewall rule for TensorBoard..."
gcloud compute firewall-rules create allow-tensorboard \
    --allow tcp:6006 \
    --target-tags tensorboard \
    --description "Allow TensorBoard access" || echo "Rule may already exist"

echo ""
echo "=== Setup Complete ==="
echo "Bucket: gs://$BUCKET_NAME"
echo "Region: $REGION"
echo ""
echo "Next: Run ./create_vm.sh to create a training VM"
