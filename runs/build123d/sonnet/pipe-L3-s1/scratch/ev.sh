#!/bin/sh
# usage: ev.sh '<json overrides>'
cd /home/user/stepbench-runs/tasks/pipe-L3-s1
P_OVR="$1" OUT=scratch/ev.step uv run --project ../../agent-env python submission.py 2>&1 | grep -E "landing|nsolids|Error|error" 
uv run --project ../../agent-env python scratch/chk.py scratch/ev.step 2>&1 | grep -E "col" | awk '{printf "%s:%s ",$1,$4}'; echo
uv run --project ../../agent-env python scratch/oh4.py 0 0 -1 scratch/ev.step 2>&1 | grep -E "total|PLANE"
uv run --project ../../agent-env python scratch/dih.py scratch/ev.step 2>&1 | grep -E "concave air angle" | head -3
