import os; os.chdir("/home/user/stepbench-runs/tasks-freecad/flange-L3-s1")
import sys
sys.argv=['x']
__file__="/home/user/stepbench-runs/tasks-freecad/flange-L3-s1/submission.py"
src=open("/home/user/stepbench-runs/tasks-freecad/flange-L3-s1/submission.py").read()
src=src.split("OUT = os.path.join")[0]
exec(src)
b0,z0=BRIDGE_C
centre_A=FA(0,b0,z0)
p=Ahole[0]
s=spoke(p,centre_A,nA,GRIP)
print("SPOKE vol", s.Volume, "valid", s.isValid(), "bb", s.BoundBox)
print("centre", centre_A, "boss", p)
