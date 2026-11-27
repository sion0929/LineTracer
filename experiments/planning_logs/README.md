# Perception logs for the planning team

One folder per Isaac run (ground-truth route replayed at 0.2 m/s, random light and texture, decoy1000 train maps 0000-0099).
Format, loading and tools: `docs/PERCEPTION_LOG.md`. Ground-truth maps: `bash experiments/worlds/make_decoy1000.sh 0`.

    python3 experiments/eval/perception_log_tool.py summary V0-NearRule640-width5-logodds/*/perception_log
    python3 experiments/eval/perception_log_tool.py rebuild V0-NearRule640-width5-logodds/0001_decoy_0000/perception_log --world --out map.png
