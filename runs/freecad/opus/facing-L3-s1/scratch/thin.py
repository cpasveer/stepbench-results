import FreeCAD as App, Part, MeshPart, time, json
V=App.Vector
s=Part.read('output.step').Solids[0]
T=json.load(open('task.json')); B=[V(*b['position']) for b in T['poses'][0]['resolved']['bolts'] if b['id'][0]=='A']
t0=time.time()
m=MeshPart.meshFromShape(Shape=s, MaxLength=0.7)
bad=[];n_t=0
for f in m.Facets:
    p=V(*[sum(c)/3 for c in zip(*f.Points)])
    if p.z>-70 or min((V(p.x,p.y,0)-V(b.x,b.y,0)).Length for b in B)>13: continue
    n=V(*f.Normal); n_t+=1
    for d in (0.2,0.45,0.75):
        if not s.isInside(p-n*d,1e-4,True):
            bad.append((d,(round(p.x,2),round(p.y,2),round(p.z,2)))); break
print('tested',n_t,'thin',len(bad),'time',round(time.time()-t0))
for b in bad[:20]: print(b)
