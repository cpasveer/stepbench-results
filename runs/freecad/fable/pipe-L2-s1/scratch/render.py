import json, math, os
import FreeCAD as App, Part
import numpy as np
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
T=json.load(open("task.json")); P=T["poses"][0]
RA=np.array(P["placements"]["A"]["rotation"]); OA=np.array(P["placements"]["A"]["origin"])
RB=np.array(P["placements"]["B"]["rotation"])
ax_=RA[:,0]; ew=RB[:,1]; eu=np.cross(ew,ax_)
M=np.stack([eu,ew,ax_])   # rows: to F coords
def mesh(shape,tol=0.3):
    v,f=shape.tessellate(tol)
    v=np.array([[p.x,p.y,p.z] for p in v]); v=(v-OA)@M.T
    return v,np.array(f)
part=mesh(Part.read("output.step")); A=mesh(Part.read("fixed_A.step"),1.0); B=mesh(Part.read("fixed_B.step"),1.0)
fig=plt.figure(figsize=(18,12))
views=[(20,-60),(20,30),(20,120),(20,210),(80,-90),(-30,-60)]
for i,(el,az) in enumerate(views):
    a=fig.add_subplot(2,3,i+1,projection="3d")
    for (v,f),c,al in ((part,"orange",1.0),(B,"lightblue",0.5)):
        tri=v[f]; n=np.cross(tri[:,1]-tri[:,0],tri[:,2]-tri[:,0]); n/= (np.linalg.norm(n,axis=1)[:,None]+1e-12)
        L=np.array([0.3,0.5,0.8]); sh=0.4+0.6*np.abs(n@L)
        base=np.array(matplotlib.colors.to_rgb(c))
        pc=Poly3DCollection(tri,facecolors=np.clip(sh[:,None]*base,0,1),edgecolors="none",alpha=al)
        a.add_collection3d(pc)
    a.set_xlim(0,120);a.set_ylim(-30,90);a.set_zlim(-60,60);a.view_init(el,az);a.set_xlabel("u");a.set_ylabel("w");a.set_zlabel("x")
plt.tight_layout();plt.savefig("scratch/render.png",dpi=60)
