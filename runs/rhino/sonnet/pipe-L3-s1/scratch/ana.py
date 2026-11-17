import numpy as np,json,itertools
T=json.load(open('./task.json'))
O=np.array([21,136.5,-7.]);R=np.array(T['poses'][0]['placements']['A']['rotation'])
Bo=np.array(T['poses'][0]['placements']['B']['origin']);Rb=np.array(T['poses'][0]['placements']['B']['rotation'])
L=lambda v:R.T@np.array(v)
c=L(Bo-O);U=L(Rb[:,0]);V=L(Rb[:,1]);N=L(Rb[:,2])
Bt=3.5
def bp(u,v,n=0): return c+u*U+v*V+n*N
# tool cylinders: (p0, axis, radius, length) in local
cyl=[]
for b in T['poses'][0]['resolved']['bolts']:
    p=L(np.array(b['position'])-O); a=L(b['axis'])
    if b['id'][0]=='A': g=4.0; hd=8.5;hh=5
    else: g=Bt; hd=7;hh=4
    cyl.append((b['id'],p+a*(g+0.0),a,hd/2+0.5,hh+1+30)) # head zone + tool
def clearance(S,E,w,h,Eclip=None):
    d=(E-S);Ln=np.linalg.norm(d);d/=Ln
    e1=V-np.dot(V,d)*d;e1/=np.linalg.norm(e1);e2=np.cross(d,e1)
    worst={}
    for s in np.linspace(0,Ln,40):
      for a in np.linspace(-w/2,w/2,9):
        for b in np.linspace(-h/2,h/2,5):
          P=S+s*d+a*e1+b*e2
          for nm,p0,ax,r,l in cyl:
            q=P-p0;t=q@ax
            if t<-0.0 or t>l: continue
            dist=np.linalg.norm(q-t*ax)-r
            if nm not in worst or dist<worst[nm]: worst[nm]=dist
    return worst
if __name__=='__main__':
    for u,v in itertools.product([-12.5,-14,-10],[-4,0,4,8]):
        E=bp(u,v,2.0);S=np.array([-4.,22.,32.])
        S2=S
        print(u,v,{k:round(x,1) for k,x in clearance(S,E,14,9).items() if x<6})
