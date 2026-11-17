import os, json
import FreeCAD as App, Part
import numpy as np
V=App.Vector
T=json.load(open('task.json'))
R=np.array(T['poses'][0]['placements']['A']['rotation']);c=np.array(T['poses'][0]['placements']['A']['origin'])
ax=V(*R[:,0])
s=Part.read(os.environ.get('STEP','output.step')).Solids[0]
for x in np.arange(-24,26,1.0):
    p=V(*(c+R[:,0]*x))
    ws=s.slice(ax,x+float(np.dot(c,R[:,0])) )
    # slice uses distance along normal from origin
    tot=0;loops=[]
    for w in ws:
        try:
            f=Part.Face(w);a=f.Area
        except Exception: a=0
        loops.append(round(a,1))
    print(round(x,1),loops)
