import Part, FreeCAD as App, math, sys
fn = globals().get("FN","output.step")
s=Part.read(fn)
side=0; hor=0
for f in s.Faces:
    # integrate normals via tessellation
    v,t=f.tessellate(0.3)
    for tri in t:
        a,b,c=[v[i] for i in tri]
        n=(b-a).cross(c-a); A=n.Length/2
        if A==0: continue
        nz=abs(n.z/n.Length)
        if nz>0.7: hor+=A
        else: side+=A
shell=1.2*side+0.8*hor
print(fn,"vol",round(s.Volume),"side",round(side),"hor",round(hor),"est",round((shell+0.2*(s.Volume-shell))/1000,2))
