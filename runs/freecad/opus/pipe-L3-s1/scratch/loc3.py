import sys
sys.argv=['submission']
exec(open('submission.py').read().split("OUT = os.path")[0])
s=Part.read('output.step').Solids[0]
p=V(26.54, 88.136, 34.51); l=to_local_pt(p)
print('local',l,'t',l.y*S.y+l.z*S.z,'v',l.dot(BV)-BCV,'r',(l.y**2+l.z**2)**.5)
f6=s.Faces[6]; n6=f6.normalAt(0,0)
for i,f in enumerate(s.Faces):
    if i==6: continue
    com=f6.section(f)
    if com.Edges:
        e=com.Edges[0]; m=e.valueAt((e.FirstParameter+e.LastParameter)/2)
        nf=f.normalAt(*f.Surface.parameter(m))
        print(i, f.Surface.__class__.__name__, 'angle between normals', round(math.degrees(n6.getAngle(nf)),1), 'len', round(sum(x.Length for x in com.Edges),1))
