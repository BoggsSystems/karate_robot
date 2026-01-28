# GCP Training Cost Estimate

## VM Options

| Instance Type | GPU | vCPUs | RAM | On-Demand | Spot Price | Speed |
|--------------|-----|-------|-----|-----------|------------|-------|
| n1-standard-4 + T4 | Tesla T4 | 4 | 15GB | $0.95/hr | $0.35/hr | 1x |
| n1-standard-8 + T4 | Tesla T4 | 8 | 30GB | $1.20/hr | $0.45/hr | 1.2x |
| n1-standard-4 + V100 | Tesla V100 | 4 | 15GB | $2.95/hr | $0.90/hr | 3x |
| a2-highgpu-1g | A100 40GB | 12 | 85GB | $4.10/hr | $1.23/hr | 5x |

## Training Time Estimates (10M steps)

| GPU | Est. Time | Spot Cost | On-Demand Cost |
|-----|-----------|-----------|----------------|
| T4 | 5-6 hours | $1.75-2.10 | $5.70-6.00 |
| V100 | 2 hours | $1.80 | $5.90 |
| A100 | 1 hour | $1.23 | $4.10 |

## Recommended Setup

**For Budget Training:**
- T4 Spot Instance: ~$2/training run
- Best value for money

**For Fast Iteration:**
- V100 Spot: ~$2/run but 3x faster
- Good for hyperparameter tuning

## Full Training Plan Cost (from training_plan.md)

| Motion | Steps | T4 Spot Cost |
|--------|-------|--------------|
| Basic Walking | 10M | $2 |
| Robust Walking | 25M | $5 |
| Basic Kicks | 5M | $1 |
| Combinations | 15M | $3 |
| **Total** | **55M** | **$11** |

## Quick Start Commands

```bash
# 1. Set up infrastructure (once)
./setup.sh

# 2. Create VM when ready to train
./create_vm.sh

# 3. Start training
./train_on_cloud.sh

# 4. Stop VM when done (stop billing!)
gcloud compute instances stop karate-training --zone=us-central1-a

# 5. Delete VM when training complete
gcloud compute instances delete karate-training --zone=us-central1-a
```

## Important: Stop Your VM!

Spot instances auto-terminate, but if using on-demand:
- **Always stop/delete VM after training**
- Set up budget alerts in GCP Console
- Use `gcloud compute instances list` to check running VMs
