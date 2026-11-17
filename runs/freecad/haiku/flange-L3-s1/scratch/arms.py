import numpy as np, sys
sys.path.insert(0,'/home/user/stepbench-runs/tasks-freecad/flange-L3-s1/scratch')
from evalD import *
def bidx(b,z):
    d=[np.linalg.norm(loc(p)[1:]-np.array([b,z])) for p in Bhole]
    return int(np.argmin(d))
def build_arms(arm_BR, arm_WALL, wA, tA, wbar, tbar, spine=True, N1=(30,16.5), N2=(8,-63.5), hub=(0,0), extra_stiff=False):
    nodes={}; elems=[]
    nodes["H"]=FA(2.5,*hub)
    for i,p in enumerate(Ahole):
        nodes[f"Afix{i}"]=p
        elems.append((f"Afix{i}","H", spoke_props(wA,tA)))
    nodes["N1"]=onB(*N1); nodes["N2"]=onB(*N2)
    Ao=(2*arm_BR)**2-(2*(arm_BR-arm_WALL))**2; Io=((2*arm_BR)**4-(2*(arm_BR-arm_WALL))**4)/12
    elems.append(("H","N1",(Ao,Io,Io,2*Io)))
    elems.append(("H","N2",(Ao,Io,Io,2*Io)))
    for i,p in enumerate(Bhole): nodes[f"Bbolt{i}"]=p
    nodes["Bc"]=OB
    for bb in [(33.7,16.5),(5.3,16.5)]:
        elems.append(("N1",f"Bbolt{bidx(*bb)}", spoke_props(wbar,tbar)))
    for bb in [(5.3,-63.5),(33.7,-63.5)]:
        elems.append(("N2",f"Bbolt{bidx(*bb)}", spoke_props(wbar,tbar)))
    if spine:
        elems.append(("N1","N2", spoke_props(wbar,tbar)))
    return nodes, elems
if __name__=="__main__":
    print("bolt idx map", [(i, np.round(loc(p)[1:],1)) for i,p in enumerate(Bhole)])
    for args in [dict(arm_BR=5,arm_WALL=1.2,wA=8,tA=8,wbar=6,tbar=5,spine=False),
                 dict(arm_BR=5,arm_WALL=1.2,wA=8,tA=8,wbar=6,tbar=5,spine=True),
                 dict(arm_BR=6,arm_WALL=1.2,wA=8,tA=8,wbar=8,tbar=6,spine=True),
                 dict(arm_BR=6,arm_WALL=1.2,wA=10,tA=8,wbar=8,tbar=8,spine=True)]:
        n,e=build_arms(**args)
        r,C=D(n,e); worst=max(r,key=r.get)
        print(args, "worst",worst, round(r[worst],3), {k:round(v,3) for k,v in r.items()})
