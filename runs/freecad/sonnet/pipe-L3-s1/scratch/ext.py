import os, math
import FreeCAD as App, Part
import numpy as np
s=Part.read(os.environ.get('STEP','output.step'))
sol=s.Solids[0]
pl=[(f.Area,f) for f in sol.Faces if f.Surface.__class__.__name__=='Plane']
pl.sort(key=lambda t:-t[0])
V=np.array([[v.X,v.Y,v.Z] for v in sol.Vertexes])
a1=np.array([0.7093118905965579,-0.6509870200868549,0.27035632165831364])
a3=np.array([-0.3771547728198181,-0.6713858946506742,0.6379617989560074])
n=np.array([0.3885173161164798,0.9214414224885092,0])
for A,f in pl[:20]:
    nm=f.normalAt(*f.Surface.parameter(f.CenterOfMass));nm=np.array([nm.x,nm.y,nm.z])
    u=-nm
    lvl=np.dot(f.CenterOfMass.x*1.0,0)
    c=np.array([f.CenterOfMass.x,f.CenterOfMass.y,f.CenterOfMass.z])
    h=(V-c)@u
    print('area %.0f normal %s extreme:%s  min %.2f max %.1f  a1.u %.2f a3.u %.2f n.u %.2f'%(A,nm.round(2),h.min()>-0.05,h.min(),h.max(),a1@u,a3@u,n@u))
