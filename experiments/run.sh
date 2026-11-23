#!/bin/bash
# usage: experiments/run.sh NAME [follower args...]   (runs one headless trial in the lab container)
name=$1; shift
sg docker -c "docker exec line_follower_lab bash -c 'source /opt/ros/jazzy/setup.bash; source /ws/install_lab/setup.bash; cd /ws && python3 experiments/run_eval.py --name $name --plot ${EVAL_ARGS:-} -- $*'" 2>&1 | grep RESULT
