import math, sys
import FreeCAD as App, Part
V=App.Vector
s=Part.read('output.step')
for D in (V(0,0,-1),V(0,0,1)):
    bed=max(v.Point.dot(D) for v in s.Vertexes)
    print('down',D, 'bed area', sum(f.Area for f in s.Faces if f.Surface.__class__.__name__=='Plane' and abs(f.CenterOfMass.dot(D)-bed)<1e-4))
    for i,f in enumerate(s.Faces):
        pts,tris=f.tessellate(0.2)
        bad=0
        for t in tris:
            a,b,c=[pts[k] for k in t]
            n=(b-a).cross(c-a); ar=n.Length/2
            if ar<1e-9: continue
            n.normalize()
            # orient with face normal
            cen=(a+b+c)*(1/3.)
            try:
                uv=f.Surface.parameter(cen); fn=f.normalAt(*uv)
                if fn.dot(n)<0: n=-n
            except Exception: pass
            if n.dot(D)>math.cos(math.radians(45))+1e-6 and abs(cen.dot(D)-bed)>1e-3: bad+=ar
        if bad>0.01: print('  face',i,f.Surface.__class__.__name__,'area',round(f.Area,2),'overhang',round(bad,2), 'z', round(f.CenterOfMass.z,2))
