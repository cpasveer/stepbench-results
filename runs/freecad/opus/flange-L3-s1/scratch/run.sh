#!/bin/sh
# usage: run.sh name '{"HA":..}'
cd /home/user/stepbench-runs/tasks-freecad/flange-L3-s1
CONN_PARAMS="$2" CONN_OUT=scratch/$1.step freecadcmd submission.py 2>&1 | grep -E "solids|Error|error" 
freecadcmd -c "FN='scratch/$1.step'; exec(open('scratch/score.py').read())" 2>&1 | tail -1
freecadcmd -c "FN='scratch/$1.step'; H=2.5; MODE='dist'; exec(open('scratch/fea.py').read())" 2>&1 | grep -E "worst" | tr '\n' ' '; echo
