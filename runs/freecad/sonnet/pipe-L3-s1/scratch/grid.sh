#!/bin/sh
cd /home/user/stepbench-runs/tasks-freecad/pipe-L3-s1
for ep in 2850 3500 4500; do for ez in 1500 2000 2850; do for gz in 0.3 0.5 1; do for gp in 0.5 1; do
out=""
for a in 0001 0002 0003; do
v=$(STEP=attempts/$a/output.step CL=4 YM=$ep EZ=$ez GZF=$gz GPF=$gp freecadcmd scratch/fea.py 2>&1 | grep eig | sed 's/.*\[ *//' | awk '{print $1}')
out="$out $v"
done
echo "Ep $ep Ez $ez Gz $gz Gp $gp : $out"
done; done; done; done
