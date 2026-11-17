import json, math
import FreeCAD as App, Part
V=App.Vector
T=json.load(open('task.json')); P=T['poses'][0]['resolved']
cA=V(*P['mounting_faces'][0]['origin']); cB=V(*P['mounting_faces'][1]['origin'])
R=T['poses'][0]['placements']['A']['rotation']
lx=V(R[0][0],R[1][0],0); ly=V(R[0][1],R[1][1],0)
def loc(p): d=V(p[0],p[1],0)-V(cA.x,cA.y,0); return (d.dot(lx), d.dot(ly))
A=[loc(b['position']) for b in P['bolts'] if b['id'][0]=='A']
B=[loc(b['position']) for b in P['bolts'] if b['id'][0]=='B']
cb=loc(cB); print('A',A); print('B',B); print('cB',cb)
def hull(pts):
    pts=sorted(set(pts))
    def cr(o,a,b): return (a[0]-o[0])*(b[1]-o[1])-(a[1]-o[1])*(b[0]-o[0])
    lo=[];up=[]
    for p in pts:
        while len(lo)>=2 and cr(lo[-2],lo[-1],p)<=0: lo.pop()
        lo.append(p)
    for p in reversed(pts):
        while len(up)>=2 and cr(up[-2],up[-1],p)<=0: up.pop()
        up.append(p)
    return lo[:-1]+up[:-1]
def face(poly):
    w=Part.makePolygon([V(x,y,0) for x,y in poly]+[V(poly[0][0],poly[0][1],0)]); return Part.Face(w)
def disc(c,r,n=48): return [(c[0]+r*math.cos(2*math.pi*i/n), c[1]+r*math.sin(2*math.pi*i/n)) for i in range(n)]
allowA=Part.makePlane(71,71,V(-35.5,-35.5,0))
allowB=Part.Face(Part.Wire(Part.makeCircle(41.25,V(cb[0],cb[1],0))))
beta=math.radians(50)
res=[]
for th in range(0,360,2):
    D=(math.cos(math.radians(th)),math.sin(math.radians(th))); Dp=(-D[1],D[0])
    SA=sum([disc(a,4.5) for a in A],[]); SB=sum([disc(b,5.25) for b in B],[])
    h=max(D[0]*x+D[1]*y for x,y in SA+SB)
    out=[]
    for S,al in ((SA,allowA),(SB,allowB)):
        pts=list(S)
        for p in S:
            t=h-(D[0]*p[0]+D[1]*p[1])
            for s in (1,-1):
                # direction down at beta: D*sin + s*Dp*cos ; reach bed
                k=t/math.sin(beta)
                pts.append((p[0]+k*(D[0]*math.sin(beta)+s*Dp[0]*math.cos(beta)), p[1]+k*(D[1]*math.sin(beta)+s*Dp[1]*math.cos(beta))))
        F=face(hull(pts)); out.append(F.cut(al).Area); out.append(F.Area)
    res.append((out[0]+out[2],th,out))
res.sort()
for r in res[:12]: print(r)
