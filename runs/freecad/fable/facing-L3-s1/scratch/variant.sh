#!/bin/sh
# usage: variant.sh NAME 'python-replacements'   (run from task folder)
NAME=$1
python3 - "$NAME" "$2" <<'PY'
import sys,json
name,reps=sys.argv[1],json.loads(sys.argv[2])
t=open("submission.py").read()
for a,b in reps:
    assert a in t,a
    t=t.replace(a,b)
t=t.replace('OUT = os.path.join(HERE, "output.step")','OUT = os.path.join(HERE, "scratch", "%s.step")'%name)
t=t.replace('HERE = os.path.dirname(os.path.abspath(__file__))','HERE = os.getcwd()')
open("scratch/var_%s.py"%name,"w").write(t)
PY
freecadcmd scratch/var_$NAME.py 2>&1 | grep -e wrote -e rror -e ssert
FE_STEP=scratch/$NAME.step FE_SIZE=6 FE_MODE=R timeout 2400 freecadcmd scratch/fe.py 2>&1 | grep -e RBE3 -e local
