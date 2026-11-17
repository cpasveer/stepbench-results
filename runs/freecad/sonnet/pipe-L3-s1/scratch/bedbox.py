import os
import FreeCAD as App, Part
import numpy as np
s=Part.read(os.environ.get('STEP','output.step')).Solids[0]
for f in s.Faces:
    if f.Surface.__class__.__name__!='Plane' or f.Area<150: continue
    nm=f.normalAt(*f.Surface.parameter(f.CenterOfMass));nm=np.array([nm.x,nm.y,nm.z])
    V=np.array([[v.X,v.Y,v.Z] for v in f.Vertexes])
    e1=np.cross(nm,[1,0,0]);
    if np.linalg.norm(e1)<0.1:e1=np.cross(nm,[0,1,0])
    e1/=np.linalg.norm(e1);e2=np.cross(nm,e1)
    P=np.stack([V@e1,V@e2],1)
    # min-area rect via rotating
    best=None
    for a in np.radians(np.arange(0,180,1)):
        R=np.array([[np.cos(a),np.sin(a)],[-np.sin(a),np.cos(a)]]);Q=P@R.T
        w,h=np.ptp(Q[:,0]),np.ptp(Q[:,1])
        if best is None or w*h<best[0]:best=(w*h,min(w,h),max(w,h))
    print('area %.0f normal %s minrect %.1f x %.1f'%(f.Area,nm.round(2),best[1],best[2]))
