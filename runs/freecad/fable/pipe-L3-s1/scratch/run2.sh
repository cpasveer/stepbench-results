#!/bin/sh
# usage: run2.sh name beam_w beam_h "PARAMS..."  -> predictor with washer-loaded plate
name=$1; bw=$2; bh=$3; shift 3
[ -f scratch/$name.step ] || PARAMS="$* BEAM_W=$bw BEAM_H=$bh OUT_NAME=scratch/$name.step" freecadcmd submission.py > scratch/$name.log 2>&1
sc=$(STEPS=scratch/$name.step freecadcmd scratch/mat.py 2>&1 | grep score)
a=$(BW=4 FE_ARGS="scratch/$name.step ${RES:-2} ${RES:-2} ${RES:-2} $bw $bh" freecadcmd scratch/fe.py 2>&1 | grep -E "WORST|diag rot" | tr '\n' ' ')
b=$(BW=5 FE_ARGS="scratch/$name.step ${RES:-2} ${RES:-2} ${RES:-2} $bw $bh" freecadcmd scratch/fe.py 2>&1 | grep -E "WORST" )
python3 - "$name" "$sc" "$a" "$b" "$*" <<'PY'
import sys,re,math
name,sc,a,b,par=sys.argv[1:6]
wa=float(a.split("E:")[1]); wb=float(b.split("E:")[1])
print("== %s | %s | %s\n   BW4 %.5f BW5 %.5f -> predicted %.2f N/mm | %s"%(name,par,sc.split("valid")[0],wa,wb,1320*math.sqrt(wa*wb),a.split("WORST")[0]))
PY
