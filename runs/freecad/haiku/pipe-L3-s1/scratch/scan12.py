exec(open("/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/scratch/scan9.py").read().split('print("A:")')[0])
print("dbg leg_ok", leg_ok((-6,0), P0+AX*6+r3*38.5), leg_ok((-6,0), P0+AX*6.0+r3*38.5))
root=CB+UB*(-6)+ZB*0+NB*4.0
L=bar(root, P0+AX*6+r3*38.5, 11.0)
print("dbg direct", round(L.common(keepc).Volume,2), "B1 tool only", round(L.common(keep[2*list(BOLT).index("B1")+1]).Volume,2))
print("dbg keep count", len(keep), "keepc vol", round(keepc.Volume,1))
