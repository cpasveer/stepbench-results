#!/bin/sh
# usage: trial.sh  (runs rhino build with current submission.py then FE)
cd .
sh run_rhino.sh | grep -E "^(error|ok|solid|small|short|Trace|  File|[A-Za-z]*Error)" | head -12
cd scratch && python3 fe.py mesh_local.json 1.0 2>&1 | grep -E "stiffness|volume|printed"
