import json, math
import FreeCAD as App, Part
V=App.Vector
s=Part.read('output.step'); print('solids',len(s.Solids),'faces',len(s.Faces),'vol',s.Volume, s.isValid())
for f in ['fixed_A.step','fixed_B.step']:
    print(f,'overlap',s.common(Part.read(f)).Volume)
T=json.load(open('task.json')); P=T['poses'][0]['resolved']
for b in P['bolts']:
    p=V(*b['position']); ax=V(*b['axis']); hd=b['head_diameter']; hh=b['head_height']
    # find grip: part thickness along axis near hole wall
    grip = 5.5 if b['id'][0]=='A' else 4.5
    top=p+ax*grip
    head=Part.makeCylinder(hd/2+0.5, hh+1, top, ax)
    tool=Part.makeCylinder(hd/2, 30, top+ax*hh, ax)
    print(b['id'], 'head', round(s.common(head).Volume,3), 'tool', round(s.common(tool).Volume,3))
# overhang: down dir = A local y
R=T['poses'][0]['placements']['A']['rotation']; D=V(R[0][1],R[1][1],0)
bed=max(v.Point.dot(D) for v in s.Vertexes)
for i,f in enumerate(s.Faces):
    u0,u1,v0,v1=f.ParameterRange; bad=0; n=0
    for a in range(5):
        for c in range(5):
            u=u0+(u1-u0)*(a+.5)/5; v=v0+(v1-v0)*(c+.5)/5
            pt=f.valueAt(u,v)
            if not f.isInside(pt,1e-3,True): continue
            nn=f.normalAt(u,v); n+=1
            if nn.dot(D)>math.cos(math.radians(45))+1e-6 and abs(pt.dot(D)-bed)>1e-3: bad+=1
    if bad: print('overhang face',i,f.Surface.__class__.__name__, round(f.Area,1), bad,'/',n)
print('bed contact faces', [round(f.Area,1) for f in s.Faces if abs(f.CenterOfMass.dot(D)-bed)<1e-3])
