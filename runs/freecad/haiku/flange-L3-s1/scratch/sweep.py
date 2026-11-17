import numpy as np, sys, itertools
sys.path.insert(0,'/home/user/stepbench-runs/tasks-freecad/flange-L3-s1/scratch')
from model import *
def build(BR, WALL, wB, tB, wA, tA, root, pad, hubB=None):
    nodes={}; elems=[]
    nodes["hub"]=FA(2.5,0,0)
    for i,p in enumerate(Ahole):
        nodes[f"Afix{i}"]=p
        elems.append((f"Afix{i}","hub", spoke_props(wA,tA)))
    nodes["S"]=FA(2.5,*root)
    elems.append(("hub","S",(1e3,1e4,1e4,1e4)))
    Ao=(2*BR)**2-(2*(BR-WALL))**2; Io=((2*BR)**4-(2*(BR-WALL))**4)/12
    nodes["P"]=onB(*pad)
    elems.append(("S","P",(Ao,Io,Io,2*Io)))
    for i,p in enumerate(Bhole):
        nodes[f"Bbolt{i}"]=p
        elems.append(("P",f"Bbolt{i}", spoke_props(wB,tB)))
    nodes["Bc"]=OB
    vol = 0
    return nodes, elems
def metrics(nodes, elems, E=2500.0):
    C = K_of(E, nodes, elems)
    # force cases: 7.85 N along unit directions in local frame (sample 3 axes + diagonals)
    dirs=[np.eye(3)[i] for i in range(3)]
    best=0; worst_d=1e9
    for d in dirs:
        disp = d@C[:3,:3]@d*7.85
        worst_d=min(worst_d, disp)
    # moment case: M = 7.85*74.54 about each axis, displacement of Bc
    Mcase=[]
    for i in range(3):
        m=np.zeros(3); m[i]=1
        Mcase.append(abs(C[:3,3:6]@m)@np.ones(3)*7.85*74.54)
    return max(d for d in [worst_d*1]), C
if __name__=="__main__":
    rows=[]
    for BR,wB,tB,root,pad in [
        (7,4.5,5,(6.9,-4),(25,-15)),
        (8,8,8,(6.9,-4),(25,-15)),
        (10,10,10,(6.9,-4),(25,-15)),
        (8,10,10,(6.9,-4),(19.5,-23.5)),
        (10,12,10,(6.9,-4),(19.5,-23.5)),
    ]:
        nodes,elems=build(BR,1.2,wB,tB,4.5,5,root,pad)
        dmax,C = metrics(nodes,elems)
        K = {}
        for i,nm in enumerate(["a","b","z"]):
            K[nm]=1/C[i,i]
        nBl=np.array([nB@nA,nB@tA,0]); Kn=1/(nBl@C[:3,:3]@nBl)
        print(f"BR={BR} wB={wB} tB={tB} pad={pad}  K(E=2500): a={K['a']:.0f} b={K['b']:.0f} z={K['z']:.0f} nB={Kn:.0f}  rot-z C={C[5,5]:.3e}")
