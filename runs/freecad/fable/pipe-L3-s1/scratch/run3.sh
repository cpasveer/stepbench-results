#!/bin/sh
# usage: run3.sh name beam_w grid_h "PARAMS..."  -> beam-limited estimate (B seat fully tied)
name=$1; bw=$2; bh=$3; shift 3
[ -f scratch/$name.step ] || PARAMS="$* BEAM_W=$bw OUT_NAME=scratch/$name.step" freecadcmd submission.py > scratch/$name.log 2>&1
sc=$(STEPS=scratch/$name.step freecadcmd scratch/mat.py 2>&1 | grep score | cut -d' ' -f2-7)
a=$(CFGS="full ${CF:-}" FE_ARGS="scratch/$name.step ${RES:-2} ${RES:-2} ${RES:-2} $bw $bh" freecadcmd scratch/fe.py 2>&1 | grep -E "CFG|rror")
echo "== $name | $* | $sc"; echo "$a"
