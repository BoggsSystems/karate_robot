#!/bin/bash
echo "=== Training Status ==="
echo ""
# Get latest log lines
if [ -f /Users/jeffboggs/karate_robot/training_output/training.log ]; then
    echo "Latest progress:"
    grep -E "(iterations|total_timesteps|ep_rew_mean|Saved)" /Users/jeffboggs/karate_robot/training_output/training.log | tail -20
    echo ""
    echo "Checkpoints saved:"
    ls -lh /Users/jeffboggs/karate_robot/training_output/*.zip 2>/dev/null || echo "No checkpoints yet"
else
    echo "Training log not found. Training may not have started yet."
fi
