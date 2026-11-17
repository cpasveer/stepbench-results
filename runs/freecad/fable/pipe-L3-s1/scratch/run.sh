#!/bin/sh
# usage: run.sh name beam_w beam_h "PARAMS..."   (run from the task folder)
name=$1; bw=$2; bh=$3; shift 3
if [ ! -f scratch/$name.step ]; then
PARAMS="$* BEAM_W=$bw BEAM_H=$bh OUT_NAME=scratch/$name.step" freecadcmd submission.py > scratch/$name.log 2>&1
fi
STEPS=scratch/$name.step freecadcmd scratch/mat.py 2>&1 | grep score >> scratch/$name.log
CM_OUT=scratch/cm_$name.npy FE_ARGS="scratch/$name.step ${RES:-2} ${RES:-2} ${RES:-2} $bw $bh" freecadcmd scratch/fe.py 2>&1 | grep -E "WORST|diag|SIX|load" >> scratch/$name.log
echo "== $name $bw $bh $*"; grep -E "score|WORST|diag|SIX|load" scratch/$name.log | tail -6
