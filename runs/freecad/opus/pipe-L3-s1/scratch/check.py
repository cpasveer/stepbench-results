import json, math, sys
import FreeCAD as App, Part
V=App.Vector
T=json.load(open('task.json')); P=T['poses'][0]
RA=P['placements']['A']['rotation']
AX=V(RA[0][0],RA[1][0],RA[2][0])
s=Part.read('output.step'); s=s.Solids[0]
print('valid',s.isValid(),'closed',s.isClosed(),'vol',round(s.Volume),'faces',len(s.Faces))
for f in ['fixed_A.step','fixed_B.step']:
    o=Part.read(f); print(f,'overlap',round(s.common(o).Volume,4))
for b in P['resolved']['bolts']:
    p=V(*b['position']); a=V(*b['axis'])
    # find hole end: grip
    grip = 5.0
    head=Part.makeCylinder(b['head_diameter']/2+0.5, b['head_height']+1, p+a*(grip+0.01), a)
    tool=Part.makeCylinder(b['head_diameter']/2, 30, p+a*(grip+b['head_height']), a)
    print(b['id'],'head',round(s.common(head).Volume,3),'tool',round(s.common(tool).Volume,3))
# overhangs, build dir
bd = -AX
if len(sys.argv)>1: bd = V(*map(float,sys.argv[1:4]))
bad=0
zmin=min(v.Point.dot(bd) for v in s.Vertexes)
for i,f in enumerate(s.Faces):
    u0,u1,v0,v1=f.ParameterRange
    worst=1
    for a in range(5):
        for c in range(5):
            u=u0+(u1-u0)*(a+0.5)/5; v=v0+(v1-v0)*(c+0.5)/5
            try:
                if not f.isInside(f.valueAt(u,v),1e-3,True): continue
            except Exception: pass
            n=f.normalAt(u,v)
            worst=min(worst,n.dot(bd))
    onbed = abs(f.CenterOfMass.dot(bd)-zmin)<1e-3
    tag=''
    if worst< -0.7071 and not onbed: tag='OVERHANG'; bad+=1
    if f.Area<1 or tag: print(i,f.Surface.__class__.__name__,round(f.Area,2),round(worst,3),'bed' if onbed else '',tag)
print('overhang faces',bad)
print('min edge',round(min(e.Length for e in s.Edges),3))
