import Part, FreeCAD as App, json, math, sys, time
import numpy as np, scipy.sparse as sp, scipy.sparse.linalg as spl
V=App.Vector
fn=globals().get("FN","output.step"); h=globals().get("H",2.0)
t0=time.time()
s=Part.read(fn)
T=json.load(open("task.json")); F={f["part"]:f for f in T["poses"][0]["resolved"]["mounting_faces"]}
cA=np.array(F["A"]["origin"]); nA=np.array(F["A"]["normal"]); cB=np.array(F["B"]["origin"]); nB=np.array(F["B"]["normal"])
bb=s.BoundBox
xs=np.arange(bb.XMin,bb.XMax+h,h); ys=np.arange(bb.YMin,bb.YMax+h,h); zs=np.arange(bb.ZMin,bb.ZMax+h,h)
nx,ny,nz=len(xs)-1,len(ys)-1,len(zs)-1
solid=np.zeros((nx,ny,nz),bool)
vv,tt=s.tessellate(0.1)
vv=np.array([[q.x,q.y,q.z] for q in vv]); tt=np.array(tt)
A_=vv[tt[:,0]];B_=vv[tt[:,1]];C_=vv[tt[:,2]]
zc=zs[:-1]+h/2
for i in range(nx):
    x=xs[i]+h/2
    for j in range(ny):
        y=ys[j]+h/2+1e-4
        # barycentric test in xy
        d=(B_[:,1]-C_[:,1])*(A_[:,0]-C_[:,0])+(C_[:,0]-B_[:,0])*(A_[:,1]-C_[:,1])
        l1=((B_[:,1]-C_[:,1])*(x-C_[:,0])+(C_[:,0]-B_[:,0])*(y-C_[:,1]))/np.where(d==0,1e-30,d)
        l2=((C_[:,1]-A_[:,1])*(x-C_[:,0])+(A_[:,0]-C_[:,0])*(y-C_[:,1]))/np.where(d==0,1e-30,d)
        l3=1-l1-l2
        m=(d!=0)&(l1>=0)&(l2>=0)&(l3>=0)
        if not m.any(): continue
        zi=np.sort(l1[m]*A_[m,2]+l2[m]*B_[m,2]+l3[m]*C_[m,2])
        cnt=np.searchsorted(zi,zc)
        solid[i,j,:]=(cnt%2)==1
print("voxels",solid.sum(),"time",round(time.time()-t0,1))
# hex8 stiffness
E=2300.;nu=0.35
def KE():
    D=E/((1+nu)*(1-2*nu))*np.array([[1-nu,nu,nu,0,0,0],[nu,1-nu,nu,0,0,0],[nu,nu,1-nu,0,0,0],[0,0,0,(1-2*nu)/2,0,0],[0,0,0,0,(1-2*nu)/2,0],[0,0,0,0,0,(1-2*nu)/2]])
    nodes=np.array([[0,0,0],[1,0,0],[1,1,0],[0,1,0],[0,0,1],[1,0,1],[1,1,1],[0,1,1]])*2-1
    K=np.zeros((24,24));g=1/math.sqrt(3)
    for a in (-g,g):
        for b in (-g,g):
            for c in (-g,g):
                dN=np.array([[nd[0]*(1+b*nd[1])*(1+c*nd[2]),nd[1]*(1+a*nd[0])*(1+c*nd[2]),nd[2]*(1+a*nd[0])*(1+b*nd[1])] for nd in nodes])/8
                J=h/2; dNx=dN/J
                B=np.zeros((6,24))
                for n in range(8):
                    B[0,3*n]=dNx[n,0];B[1,3*n+1]=dNx[n,1];B[2,3*n+2]=dNx[n,2]
                    B[3,3*n]=dNx[n,1];B[3,3*n+1]=dNx[n,0]
                    B[4,3*n+1]=dNx[n,2];B[4,3*n+2]=dNx[n,1]
                    B[5,3*n]=dNx[n,2];B[5,3*n+2]=dNx[n,0]
                K+=B.T@D@B*J**3
    return K
Ke=KE()
off=np.array([[0,0,0],[1,0,0],[1,1,0],[0,1,0],[0,0,1],[1,0,1],[1,1,1],[0,1,1]])
el=np.argwhere(solid)
nid=lambda ijk: (ijk[...,0]*(ny+1)+ijk[...,1])*(nz+1)+ijk[...,2]
en=nid(el[:,None,:]+off[None,:,:])  # (ne,8)
used,inv=np.unique(en,return_inverse=True); inv=inv.reshape(en.shape)
nn=len(used)
dof=(inv[:,:,None]*3+np.arange(3)).reshape(len(el),24)
rows=np.repeat(dof,24,axis=1).ravel(); cols=np.tile(dof,(1,24)).ravel()
vals=np.tile(Ke.ravel(),len(el))
K=sp.coo_matrix((vals,(rows,cols)),shape=(3*nn,3*nn)).tocsr()
# node coordinates
ii=used//((ny+1)*(nz+1)); jj=(used//(nz+1))%(ny+1); kk=used%(nz+1)
P=np.stack([xs[0]+ii*h, ys[0]+jj*h, zs[0]+kk*h],1)
wA=(P-cA)@nA; dB=(P-cB)@nB
fixed=np.where(wA<h*0.75)[0]
load=np.where(dB<h*0.75)[0]
print("nodes",nn,"fixed",len(fixed),"load",len(load))
MODE=globals().get("MODE","rigid")
span=74.54
if MODE=="dist":
    free=np.ones(3*nn,bool); free[(fixed[:,None]*3+np.arange(3)).ravel()]=False
    Kf=K[free][:,free].tocsc(); lu=spl.splu(Kf)
    L=load[~np.isin(load,fixed)]; cen=P[L].mean(0); r=P[L]-cen
    # rigid-fit matrix G: u_L = G q  (q = [U, theta])
    G=np.zeros((3*len(L),6))
    for a in range(3): G[a::3,a]=1
    G[0::3,4]=r[:,2]; G[0::3,5]=-r[:,1]; G[1::3,5]=r[:,0]; G[1::3,3]=-r[:,2]; G[2::3,3]=r[:,1]; G[2::3,4]=-r[:,0]
    Gp=np.linalg.pinv(G)
    C=np.zeros((6,6))
    for a in range(6):
        if a<3:
            f=np.zeros((nn,3)); f[L,a]=1/len(L)
        else:
            d=np.eye(3)[a-3]; fv=np.cross(d,r); m=np.sum(np.cross(r,fv)@d); fv/=m
            f=np.zeros((nn,3)); f[L]=fv
        x=np.zeros(3*nn); x[free]=lu.solve(f.ravel()[free])
        C[:,a]=Gp@x.reshape(nn,3)[L].ravel()
    kF=1/np.linalg.svd(C[:3,:3],compute_uv=False)[0]
    kM=1/(np.linalg.svd(C[3:,3:],compute_uv=False)[0]*span*span)
    print("worst-dir force k",round(kF,1),"worst moment k",round(kM,1),"(dist)")
    U,S,Vt=np.linalg.svd(C[3:,3:]); print("worst moment axis",np.round(Vt[0],2), "kM per axis",[round(1/(C[3+a,3+a]*span*span),1) for a in range(3)])
    U,S,Vt=np.linalg.svd(C[:3,:3]); print("worst force dir",np.round(Vt[0],2))
    sys.stdout.flush(); import os; os._exit(0)
# reduced dofs: fixed removed, load nodes slaved to rigid master (6 dofs)
isfix=np.zeros(nn,bool); isfix[fixed]=True
isload=np.zeros(nn,bool); isload[load]=True; isload&=~isfix
other=np.where(~isfix&~isload)[0]
nfree=3*len(other)
cen=P[isload].mean(0)
rows=[];cols=[];vals=[]
for a in range(3):
    rows.append(other*3+a); cols.append(np.arange(len(other))*3+a); vals.append(np.ones(len(other)))
L=np.where(isload)[0]; r=P[L]-cen
for a in range(3):
    rows.append(L*3+a); cols.append(np.full(len(L),nfree+a)); vals.append(np.ones(len(L)))
# theta x r : components
# x: th_y*r_z - th_z*r_y ; y: th_z*r_x - th_x*r_z ; z: th_x*r_y - th_y*r_x
for (comp,th,coef) in [(0,1,r[:,2]),(0,2,-r[:,1]),(1,2,r[:,0]),(1,0,-r[:,2]),(2,0,r[:,1]),(2,1,-r[:,0])]:
    rows.append(L*3+comp); cols.append(np.full(len(L),nfree+3+th)); vals.append(coef)
Tm=sp.coo_matrix((np.concatenate(vals),(np.concatenate(rows),np.concatenate(cols))),shape=(3*nn,nfree+6)).tocsc()
Kr=(Tm.T@K@Tm).tocsc()
lu=spl.splu(Kr)
span=74.54; res=[]
for a in range(3):
    f=np.zeros(nfree+6); f[nfree+a]=1; x=lu.solve(f); res.append(("F",a,1/x[nfree+a]))
for a in range(3):
    f=np.zeros(nfree+6); f[nfree+3+a]=span; x=lu.solve(f); res.append(("M",a,1/(x[nfree+3+a]*span)))
C=np.zeros((6,6))
for a in range(6):
    f=np.zeros(nfree+6); f[nfree+a]=1; C[:,a]=lu.solve(f)[nfree:]
CFF=C[:3,:3]; CMM=C[3:,3:]
kF=1/np.linalg.svd(CFF,compute_uv=False)[0]
kM=1/(np.linalg.svd(CMM,compute_uv=False)[0]*span*span)
print("worst-dir force k",round(kF,1),"worst moment k",round(kM,1))
for q in res: print(q[0],q[1],round(q[2],1))
print("min",round(min(q[2] for q in res),1),"time",round(time.time()-t0,1))
