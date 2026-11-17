import os, Part
for p in ('attempts/0001/output.step','attempts/0002/output.step','attempts/0003/output.step'):
    s=Part.read(p)
    pl=[f for f in s.Faces if f.Surface.__class__.__name__=='Plane']
    areas=sorted([round(f.Area,1) for f in pl],reverse=True)
    print(p,len(s.Faces),'planar',len(pl))
    print(areas)
    # distinct normals
    import collections
    dn=collections.OrderedDict()
    for f in pl:
        n=f.normalAt(*f.Surface.parameter(f.CenterOfMass))
        key=tuple(round(x,2) for x in (n.x,n.y,n.z))
        dn[key]=dn.get(key,0)+f.Area
    print(len(dn),'distinct planar normals')
