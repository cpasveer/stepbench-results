exec(open("/home/user/stepbench-runs/tasks-freecad/flange-L2-s1/scratch/overhang.py").read().split("if __name__")[0])
for name, u in [("+X",V(1,0,0)),("-X",V(-1,0,0)),("+Y",V(0,1,0)),("-Y",V(0,-1,0)),("+Z",V(0,0,1)),("-Z",V(0,0,-1))]:
    print("AX %s %.1f" % (name, overhang(u)))
