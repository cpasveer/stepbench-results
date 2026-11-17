#!/bin/sh
# usage: ev.sh '{json}'
cd /home/user/stepbench-runs/tasks-freecad/pipe-L3-s1
PRM="$1" OUTSTEP=scratch/v.step freecadcmd submission.py 2>&1 | grep -E "Trace|Error|solids|keepout overlap [1-9]|keepout overlap 0\.[0-9]*[1-9]|overlap [0-9]*\.[0-9]*[1-9]"
STEP=scratch/v.step CL=${CL:-3} YM=2850 freecadcmd scratch/fea.py 2>&1 | grep -E "eig"
