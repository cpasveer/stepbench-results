import Part, FreeCAD as App, math, sys
import matplotlib; matplotlib.use("Agg")
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection
import numpy as np
fig=plt.figure(figsize=(14,7))
shapes=[(Part.read(globals().get("FN","output.step")),"orange"),(Part.read("fixed_A.step"),"lightblue"),(Part.read("fixed_B.step"),"lightgreen")]
for i,(el,az) in enumerate([(25,-60),(25,150)]):
    ax=fig.add_subplot(1,2,i+1,projection="3d")
    for sh,col in shapes:
        v,f=sh.tessellate(0.5)
        v=np.array([[p.x,p.y,p.z] for p in v])
        tris=[v[list(t)] for t in f]
        ls=np.array([0.3,-0.5,0.8]);ls/=np.linalg.norm(ls)
        cols=[]
        for t in tris:
            n=np.cross(t[1]-t[0],t[2]-t[0]);nn=np.linalg.norm(n)
            k=0.4+0.6*abs(np.dot(n/nn if nn else n,ls))
            c=np.array(matplotlib.colors.to_rgb(col))*k
            cols.append(np.clip(c,0,1))
        pc=Poly3DCollection(tris,facecolors=cols,edgecolor="none")
        ax.add_collection3d(pc)
    ax.set_xlim(50,170);ax.set_ylim(-130,10);ax.set_zlim(-70,50)
    ax.set_box_aspect((120,140,120))
    ax.view_init(el,az)
plt.tight_layout();plt.savefig("scratch/render.png",dpi=70)
# second figure: views along A normal (from B side) and along B normal
fig=plt.figure(figsize=(14,7))
s=Part.read(globals().get("FN","output.step"))
import FreeCAD as App
for i,(n,c) in enumerate([((0.7384553406258838,0.6743023875837234,0),(95,-91,5.5)),((-0.4786918579404784,-0.8779829754276512,0),(132.07,-30.75,-18))]):
    ax=fig.add_subplot(1,2,i+1)
    n=np.array(n); u=np.array([-n[1],n[0],0]); 
    v,f=s.tessellate(0.3); v=np.array([[p.x,p.y,p.z] for p in v])
    d=(v-np.array(c))@n
    for t in f:
        p=v[list(t)]; dd=d[list(t)].mean()
        ax.fill((p-np.array(c))@u, p[:,2], color=plt.cm.viridis(min(1,max(0,dd/70))), lw=0)
    ax.set_aspect("equal"); ax.set_title("view along -n of %s (colour = distance from face)"%("A" if i==0 else "B"))
plt.savefig("scratch/render2.png",dpi=60)
