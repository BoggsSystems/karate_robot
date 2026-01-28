# Cloud Training Setup

Ready-to-use scripts for training on cloud GPUs.

## GCP (Google Cloud Platform)

### Prerequisites
1. Install [Google Cloud SDK](https://cloud.google.com/sdk/docs/install)
2. Run `gcloud auth login`
3. Project already created: `robottraining-485701`

### Fix Permissions Error
In GCP Console → IAM & Admin → IAM:
1. Find your email and click Edit (pencil icon)
2. Add roles:
   - `Logs Viewer`
   - `Compute Admin` 
   - `Storage Admin`

### Quick Start

```bash
cd cloud/gcp

# 1. One-time setup (enable APIs, create bucket)
./setup.sh

# 2. Create GPU VM (~$0.35/hr with Spot pricing)
./create_vm.sh

# 3. Upload code and start training
./train_on_cloud.sh

# 4. IMPORTANT: Stop VM when done!
gcloud compute instances stop karate-training --zone=us-central1-a
```

### Cost Summary
- **T4 GPU Spot**: ~$0.35/hr → $2 for 10M steps
- **Full training plan**: ~$11 total
- Always use Spot instances for 60-70% savings

### Monitor Training
```bash
# SSH into VM
gcloud compute ssh karate-training --zone=us-central1-a

# View TensorBoard (from VM)
tensorboard --logdir=/output --bind_all

# Download checkpoints
gsutil cp gs://robottraining-485701-training/checkpoints/* ./
```

## Azure (Alternative)

Similar setup available. Use Azure ML or GPU VMs with Spot pricing.
Comparable costs to GCP.
