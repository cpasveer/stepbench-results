import json, math, itertools
T=json.load(open('task.json')); P=T['poses'][0]
R=P['placements']['A']['rotation']; C=P['placements']['A']['origin']
col=lambda i:[R[0][i],R[1][i],R[2][i]]
d,yl,zl=col(0),col(1),col(2)
OB=P['resolved']['mounting_faces'][0]['origin']; n=P['resolved']['mounting_faces'][0]['normal']
u=[n[1],-n[0],0.0]; zw=[0,0,1]
add=lambda a,b,s=1:[a[i]+s*b[i] for i in range(3)]
dot=lambda a,b:sum(a[i]*b[i] for i in range(3))
def segdist(p1,q1,p2,q2,N=200):
    best=1e9
    for i in range(N+1):
        a=add(p1,add(q1,p1,-1),i/N)
        # closest on seg2
        v=add(q2,p2,-1); t=max(0,min(1,dot(add(a,p2,-1),v)/dot(v,v)))
        b=add(p2,v,t); best=min(best,math.dist(a,b))
    return best
for u0,z0,phi,t0 in itertools.product([0,2.5,4],[0,2],[0,-8,-15],[0,-5]):
    S=add(add(OB,u,u0),zw,z0)
    ph=math.radians(phi)
    Tp=add(add(add(C,zl,34*math.cos(ph)),yl,34*math.sin(ph)),d,t0)
    v=add(Tp,S,-1); L=math.sqrt(dot(v,v)); dr=[x/L for x in v]
    radial=add(add([0,0,0],zl,math.cos(ph)),yl,math.sin(ph))
    out=[]
    for b in P['resolved']['bolts']:
        p=b['position'];a=b['axis']
        out.append(round(segdist(p,add(p,a,40),S,Tp),1))
    print(u0,z0,phi,t0,'L%.0f'%L,'angB %.0f angA %.0f'%(math.degrees(math.acos(dot(dr,n))),math.degrees(math.acos(-dot(dr,radial)))),out)
