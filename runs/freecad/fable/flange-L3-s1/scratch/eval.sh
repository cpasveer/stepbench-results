#!/bin/sh
# usage: scratch/eval.sh name 'json params' [H]
n=$1; p=$2; h=${3:-2.0}
OUT_STEP=$PWD/scratch/c_$n.step PARAMS="$p" freecadcmd submission.py > scratch/c_$n.log 2>&1
STEP=scratch/c_$n.step freecadcmd -c "exec(open('scratch/mat.py').read())" 2>&1 | grep MAT >> scratch/c_$n.log
STEP=scratch/c_$n.step H=$h FCORE=0.2 HYP=H3 timeout 1500 freecadcmd -c "exec(open('scratch/vfe.py').read())" 2>&1 | grep -E "^H" >> scratch/c_$n.log
echo "$n $p" >> scratch/c_$n.log
