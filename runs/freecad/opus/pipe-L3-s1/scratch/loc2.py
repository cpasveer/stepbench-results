import sys
sys.argv=['submission']
exec(open('submission.py').read().split("OUT = os.path")[0])
for p in [V(27.489, 51.987, 44.155), V(18.484, 105.757, 23.47)]:
    l=to_local_pt(p); print(l, 't', l.y*S.y+l.z*S.z, 'v', l.dot(BV)-BCV, 'u', (l-BC).dot(BU),'n',(l-BC).dot(BN))
s=Part.read('output.step').Solids[0]
f=s.Faces[6]; print(f.Surface, f.Area, f.CenterOfMass)
