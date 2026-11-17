import os, json
import FreeCAD as App, Part
import numpy as np
V=App.Vector
T=json.load(open('task.json'))
R=np.array(T['poses'][0]['placements']['A']['rotation']);c=np.array(T['poses'][0]['placements']['A']['origin'])
ax=V(*R[:,0])
s=Part.read(os.environ.get('STEP','scratch/v.step')).Solids[0]
lo,hi=-19.5,7.5
worst=(1e9,None)
for x in np.arange(lo,hi+0.01,0.5):
    ws=s.slice(ax,x+float(np.dot(c,R[:,0])))
    ar=[]
    for w in ws:
        try: ar.append(Part.Face(w).Area)
        except Exception: pass
    m=max(ar) if ar else 0
    if m<worst[0]: worst=(m,x)
print('SECMIN max-loop worst',round(worst[0],1),'at x',worst[1])
