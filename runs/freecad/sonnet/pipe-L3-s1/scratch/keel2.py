import os, math
import FreeCAD as App, Part
import numpy as np
u=np.array([0.24879564,0.59006536,0.76806484])
sol=Part.read(os.environ.get('STEP','output.step')).Solids[0]
for fi in (5,6,18,20,26):
    try:
        f=sol.Faces[fi]
    except: continue
    uu,vv=f.Surface.parameter(f.CenterOfMass);nm=f.Surface.normal(uu,vv)
    if f.Orientation=='Reversed': nm=-nm
    n=np.array([nm.x,nm.y,nm.z])
    print(fi,f.Surface.__class__.__name__,'area %.0f'%f.Area,'n.u %.2f'%(n@u),'overhang deg from vertical %.0f'%(90-math.degrees(math.acos(max(-1,min(1,-(n@u)))))) if n@u<0 else 'up')
