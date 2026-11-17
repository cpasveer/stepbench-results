import json,math,sys,numpy as np
from build123d import *
T=json.load(open('task.json'));P=T['poses'][0]['resolved']
part=import_step(sys.argv[1] if len(sys.argv)>1 else 'output.step')
def cylseg(r,p0,p1):
    p0,p1=Vector(*p0),Vector(*p1);ax=p1-p0
    return Part(Cylinder(r,ax.length).moved(Plane(origin=(p0+p1)*0.5,z_dir=ax.normalized()).location))
for b in P['bolts']:
    p=np.array(b['position']);a=np.array(b['axis'])
    hd=b['head_diameter'];hh=b['head_height']
    # column from seat plane: unknown grip; take from 2.5 above surface
    g0=4.2 if b["id"][0]=="A" else 3.6; col=cylseg(hd/2+0.5,p+a*(g0+0.01),p+a*(g0+hh+31))
    r=(part&col); v=r.volume if r else 0
    # key-ish: nothing
    print(b['id'],'column overlap',round(v,3))
# faces
fs=part.faces()
print('faces',len(fs),'min area',min(f.area for f in fs))
print('small faces',[round(f.area,2) for f in fs if f.area<2])
es=part.edges();print('min edge',min(e.length for e in es))
# overhang
from build123d import Mesher
vs=[];ns=[];ar=[]
for f in fs:
    vt,tr=f.tessellate(0.2)
    for t in tr:
        a=np.array([(vt[i].X,vt[i].Y,vt[i].Z) for i in t]);n=np.cross(a[1]-a[0],a[2]-a[0]);A=np.linalg.norm(n)/2
        if A>0: ns.append(n/np.linalg.norm(n));ar.append(A)
ns=np.array(ns);ar=np.array(ar)
best=[]
rng=np.random.default_rng(0)
for w in rng.normal(size=(4000,3)):
    w/=np.linalg.norm(w)
    c=ns@w
    # overhang: facing down more than 45deg from vertical wall => c<-0.707
    oh=ar[c<-0.7071].sum()
    best.append((oh,tuple(np.round(w,2))))
best.sort();print(best[:5])
print('total area',ar.sum())
