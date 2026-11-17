import numpy as np, sys
sys.path.insert(0,'/home/user/stepbench-runs/tasks-freecad/flange-L3-s1/scratch')
from model import *
def build_current(w_spoke=4.5, G_A=5.0, G_B=5.0, BR=7.0, WALL=1.2, hub_bz=(0,0), root=(6.9,-4.0), pad=(25.0,-15.0), closed=True):
    nodes={}; elems=[]
    nodes["hub"]=FA(2.5,*hub_bz)
    for i,p in enumerate(Ahole):
        nodes[f"Afix{i}"]=p
        elems.append((f"Afix{i}","hub", spoke_props(w_spoke,G_A)))
    nodes["S"]=FA(2.5,*root)
    elems.append(("hub","S",(1e3,1e4,1e4,1e4)))   # rigid junction
    if closed:
        Ao = (2*BR)**2-(2*(BR-WALL))**2; Io=((2*BR)**4-(2*(BR-WALL))**4)/12
    else:
        Ao=(2*BR)**2; Io=(2*BR)**4/12
    nodes["P"]=onB(*pad)
    elems.append(("S","P",(Ao,Io,Io,2*Io)))
    for i,p in enumerate(Bhole):
        nodes[f"Bbolt{i}"]=p
        elems.append(("P",f"Bbolt{i}", spoke_props(w_spoke,G_B)))
    nodes["Bc"]=OB
    return nodes, elems
def K_along_nB(C):
    nBl = np.array([nB@nA, nB@tA, 0])
    return 1/(nBl@C[:3,:3]@nBl)
if __name__=="__main__":
    nodes, elems = build_current()
    C = K_of(2500.0, nodes, elems)
    print("K along nB at E=2500:", K_along_nB(C), " diag K a,b,z:", [1/C[i,i] for i in range(3)])
