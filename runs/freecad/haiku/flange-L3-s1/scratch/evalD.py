import numpy as np, sys
sys.path.insert(0,'/home/user/stepbench-runs/tasks-freecad/flange-L3-s1/scratch')
from model import *
F=7.85; M=F*74.54
def build3(BR, WALL, wB, tB, wA, tA, root, pad, stiffA=False, Bframe=None):
    nodes={}; elems=[]
    nodes["hub"]=FA(2.5,*root)
    for i,p in enumerate(Ahole):
        nodes[f"Afix{i}"]=p
        pr=(1e6,1e6,1e6,1e6) if stiffA else spoke_props(wA,tA)
        elems.append((f"Afix{i}","hub",pr))
    nodes["P"]=onB(*pad)
    Ao=(2*BR)**2-(2*(BR-WALL))**2; Io=((2*BR)**4-(2*(BR-WALL))**4)/12
    elems.append(("hub","P",(Ao,Io,Io,2*Io)))
    for i,p in enumerate(Bhole):
        nodes[f"Bbolt{i}"]=p
        elems.append(("P",f"Bbolt{i}", spoke_props(wB,tB)))
    nodes["Bc"]=OB
    return nodes, elems
def D(nodes, elems):
    C=K_of(2500.0,nodes,elems)
    res={}
    for ax,name in enumerate(["a","b","z"]):
        f=np.zeros(6); f[ax]=F
        m=np.zeros(6); m[3+ax]=M
        for lab,l in [("F"+name,f),("M"+name,m)]:
            u=C@l; t=u[:3]; th=u[3:]
            res[lab]=max(np.linalg.norm(t+np.cross(th,loc(p)-loc(OB))) for p in Bhole)
    return res, C
if __name__=="__main__":
    for BR,wA,tA,wB,tB,stA in [
        (10,8,8,8,8,False),(10,8,8,8,10,False),(10,8,8,10,10,False),
        (10,8,8,8,8,True),(10,8,8,10,10,True),(10,8,8,12,12,True),
        (10,10,10,10,10,True),(8,8,8,8,8,True),]:
        nodes,elems=build3(BR,1.2,wB,tB,wA,tA,(-1,-3),(-1,-3),stA)
        r,C=D(nodes,elems)
        worst=max(r,key=r.get)
        print(f"BR{BR} A{wA}x{tA} stiffA={stA} B{wB}x{tB}: worst {worst}={r[worst]:.3f}  " + " ".join(f"{k}={v:.3f}" for k,v in r.items()))
