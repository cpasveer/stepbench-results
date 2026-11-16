import sys
sys.argv=['submission']
exec(open('submission.py').read().split("OUT = os.path")[0])
s=Part.read('output.step').Solids[0]
for i,f in enumerate(s.Faces):
    c=to_local_pt(f.CenterOfMass)
    n=f.normalAt(*f.Surface.parameter(f.CenterOfMass)) if hasattr(f.Surface,'parameter') else None
    print(i,f.Surface.__class__.__name__,round(f.Area),'x%.1f t%.1f v%.1f'%(c.x,c.y*S.y+c.z*S.z,c.dot(BV)-BCV), to_local_vec(n) if n else '')
