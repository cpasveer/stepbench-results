import os, math, FreeCAD as App, Part
sh=Part.read('output.step')
print('solids',len(sh.Solids),'valid',sh.isValid(),'vol',round(sh.Volume))
for f in ('fixed_A.step','fixed_B.step'):
    print(f,'common',round(sh.common(Part.read(f)).Volume,4))
al=math.radians(float(os.environ.get('ALPHA','30')))
xa=App.Vector(0.24879564,0.59006536,0.76806484); e2=App.Vector(0.92144142,-0.38851732,0)
u=xa*math.cos(al)-e2*math.sin(al); u.normalize()
print('bbox height along u', max(v.Point.dot(u) for v in sh.Vertexes)-min(v.Point.dot(u) for v in sh.Vertexes))
for i,f in enumerate(sh.Faces):
    if f.Area<3: print('small face',i,round(f.Area,2),f.CenterOfMass)
for e in sh.Edges:
    if e.Length<1: print('short edge',round(e.Length,2),e.CenterOfMass)
def fn(f):
    uv=f.Surface.parameter(f.CenterOfMass) if f.Surface.__class__.__name__!='Plane' else (0,0)
    p=f.CenterOfMass
    try:
        uv=f.Surface.parameter(p)
    except Exception: pass
    return f.normalAt(*uv)
dots=[fn(f).dot(u) for f in sh.Faces]
bed=min(range(len(dots)),key=lambda i:dots[i])
print('bed face',bed,round(dots[bed],3),round(sh.Faces[bed].Area))
print('down faces',[(i,round(d,2),round(sh.Faces[i].Area)) for i,d in enumerate(dots) if d<-0.707 and i!=bed])
# keels
em={}
for i,f in enumerate(sh.Faces):
    for e in f.Edges:
        for j,E in enumerate(sh.Edges):
            if e.isSame(E): em.setdefault(j,[]).append(i)
for j,fs in em.items():
    if len(fs)==2 and all(dots[i]<-0.05 and i!=bed for i in fs):
        E=sh.Edges[j]
        if E.Length<2: continue
        d=E.tangentAt(E.FirstParameter if E.Curve.__class__.__name__=='Line' else E.FirstParameter)
        el=math.degrees(math.asin(min(1,abs(d.dot(u)))))
        if el<26.6: print('KEEL',round(E.Length,1),'elev',round(el,1),fs,[round(dots[i],2) for i in fs])
f=sh.Faces[bed]
import itertools
pts=[v.Point for v in f.Vertexes]
best=None
for ang in range(0,180,2):
    a=math.radians(ang)
    ax=u.cross(App.Vector(1,0,0)); ax.normalize(); ay=u.cross(ax)
    d1=ax*math.cos(a)+ay*math.sin(a); d2=u.cross(d1)
    w=[p.dot(d1) for p in pts]; h=[p.dot(d2) for p in pts]
    W=max(w)-min(w); H=max(h)-min(h)
    if best is None or W*H<best[0]: best=(W*H,W,H)
print('bed min rect',best)
