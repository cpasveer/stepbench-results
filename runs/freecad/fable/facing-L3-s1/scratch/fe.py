"""Home-made FE check (quadratic tets, netgen mesh, scipy).  Usage:
   FE_STEP=path FE_SIZE=4 freecadcmd scratch/fe.py"""
import os, json, time, math
import numpy as np, scipy.sparse as sp, scipy.sparse.linalg as spl
import FreeCAD as App, Part, Fem, ObjectsFem
STEP=os.environ.get("FE_STEP","output.step"); SIZE=float(os.environ.get("FE_SIZE","4"))
E_MOD=float(os.environ.get("FE_E","2500")); NU=0.35
T=json.load(open("task.json")); P=T["poses"][0]; BOLTS=P["resolved"]["bolts"]
RA_=np.array(P["placements"]["A"]["rotation"])
SPAN=T["stiffness"]["per_mount"][0]["span_mm"]
t0=time.time()
doc=App.newDocument("m"); s=Part.read(STEP)
o=doc.addObject("Part::Feature","p"); o.Shape=s
m=ObjectsFem.makeMeshNetgenLegacy(doc,"mesh"); m.Shape=o; m.MaxSize=SIZE; m.MinSize=0.4; m.SecondOrder=True; m.Fineness="Moderate"
doc.recompute(); fm=m.FemMesh
nodes=fm.Nodes; ids=sorted(nodes.keys()); idx={n:i for i,n in enumerate(ids)}
X=np.array([[nodes[n].x,nodes[n].y,nodes[n].z] for n in ids])
el=np.array([[idx[n] for n in fm.getElementNodes(e)] for e in fm.Volumes])
print("mesh",len(ids),"nodes",len(el),"tets",round(time.time()-t0,1),"s",flush=True)
# reorder mid nodes
edges=[(0,1),(1,2),(2,0),(0,3),(1,3),(2,3)]
C=X[el[:,:4]]; M=X[el[:,4:]]
new=np.zeros((len(el),6),int)
for k,(a,b) in enumerate(edges):
    mid=(C[:,a]+C[:,b])/2
    d=np.linalg.norm(M-mid[:,None,:],axis=2)
    new[:,k]=el[np.arange(len(el)),4+d.argmin(1)]
el=np.hstack([el[:,:4],new])
# gauss points
a_,b_=0.5854101966249685,0.1381966011250105
GP=np.array([[a_,b_,b_,b_],[b_,a_,b_,b_],[b_,b_,a_,b_],[b_,b_,b_,a_]])
def dN(L):
    # derivatives wrt L1..L4 (10x4)
    d=np.zeros((10,4))
    for i in range(4): d[i,i]=4*L[i]-1
    for k,(a,b) in enumerate(edges):
        d[4+k,a]=4*L[b]; d[4+k,b]=4*L[a]
    # eliminate L4 = 1-L1-L2-L3 -> derivatives wrt L1..L3
    return d[:,:3]-d[:,3:4]
lam=E_MOD*NU/((1+NU)*(1-2*NU)); mu=E_MOD/(2*(1+NU))
D=np.zeros((6,6)); D[:3,:3]=lam; D[np.arange(3),np.arange(3)]+=2*mu; D[3:,3:]=np.eye(3)*mu
ne=len(el); Ke=np.zeros((ne,30,30)); XE=X[el]
for g in range(4):
    dn=dN(GP[g])                         # 10x3
    J=np.einsum('eni,nj->eij',XE,dn)     # dx_i/dxi_j
    det=np.linalg.det(J); Ji=np.linalg.inv(J)
    G=np.einsum('nj,eji->eni',dn,Ji)     # dN/dx  (e,10,3)
    B=np.zeros((ne,6,30))
    B[:,0,0::3]=G[:,:,0]; B[:,1,1::3]=G[:,:,1]; B[:,2,2::3]=G[:,:,2]
    B[:,3,0::3]=G[:,:,1]; B[:,3,1::3]=G[:,:,0]
    B[:,4,1::3]=G[:,:,2]; B[:,4,2::3]=G[:,:,1]
    B[:,5,0::3]=G[:,:,2]; B[:,5,2::3]=G[:,:,0]
    Ke+=np.einsum('eki,kl,elj,e->eij',B,D,B,np.abs(det)/24.0,optimize=True)
dof=(el[:,:,None]*3+np.arange(3)).reshape(ne,30)
I=np.repeat(dof,30,axis=1).ravel(); Jc=np.tile(dof,(1,30)).ravel()
N=len(ids)*3
K=sp.csr_matrix((Ke.ravel(),(I,Jc)),shape=(N,N))
print("assembled",round(time.time()-t0,1),"s",flush=True)
zA=P["resolved"]["mounting_faces"][0]["origin"][2]; zB=P["resolved"]["mounting_faces"][1]["origin"][2]
cB=np.array(P["placements"]["B"]["origin"])
def ring_nodes(b,face_only=False):
    p=np.array(b["position"]); r=np.linalg.norm(X[:,:2]-p[:2],axis=1)
    rh=b["clearance_diameter_range"][1]/2+0.05
    onface=(np.abs(X[:,2]-p[2])<1e-2)&(r<=b["head_diameter"]/2+1e-6)
    grip=6.0
    hole=(r<=rh)&(np.abs(X[:,2]-p[2])<grip)
    return np.where(onface|hole)[0]
def seat_nodes(z): return np.where(np.abs(X[:,2]-z)<1e-2)[0]
Ab=[b for b in BOLTS if b["id"][0]=="A"]; Bb=[b for b in BOLTS if b["id"][0]=="B"]
def solve(fixed_nodes, bodies):
    """bodies: list of (ref_point, node_idx).  returns compliance matrix (6nb x 6nb) at refs"""
    nb=len(bodies); tied=np.concatenate([b[1] for b in bodies])
    isfix=np.zeros(len(ids),bool); isfix[fixed_nodes]=True
    istied=np.zeros(len(ids),bool); istied[tied]=True
    free=np.where(~isfix&~istied)[0]; nf=len(free)*3
    rows=[];cols=[];vals=[]
    fd=(free[:,None]*3+np.arange(3)).ravel()
    rows+=list(fd); cols+=list(range(nf)); vals+=[1.0]*nf
    for k,(ref,nd) in enumerate(bodies):
        base=nf+6*k
        for n in nd:
            r=X[n]-ref
            for i in range(3):
                rows.append(3*n+i); cols.append(base+i); vals.append(1.0)
            # u = U + th x r
            rows+= [3*n+0,3*n+0,3*n+1,3*n+1,3*n+2,3*n+2]
            cols+= [base+4,base+5,base+3,base+5,base+3,base+4]
            vals+= [r[2],-r[1],-r[2],r[0],r[1],-r[0]]
    Tm=sp.csr_matrix((vals,(rows,cols)),shape=(N,nf+6*nb))
    Kr=(Tm.T@K@Tm).tocsc()
    F=np.zeros((nf+6*nb,6*nb)); F[nf:,:]=np.eye(6*nb)
    lu=spl.splu(Kr)
    U=lu.solve(F)
    return U[nf:,:]
fixA_rings=np.unique(np.concatenate([ring_nodes(b) for b in Ab]))
fixA_seat=np.unique(np.concatenate([seat_nodes(zA)]+[ring_nodes(b) for b in Ab]))
def report(name,Cm):
    lin=[1/Cm[i,i] for i in range(3)]; rot=[1/(Cm[i,i]*SPAN**2) for i in range(3,6)]
    # worst direction: smallest eigen-stiffness of the 3x3 blocks
    A_=np.linalg.inv(RA_); 
    Cl=np.block([[A_@Cm[:3,:3]@A_.T,A_@Cm[:3,3:]@A_.T],[A_@Cm[3:,:3]@A_.T,A_@Cm[3:,3:]@A_.T]])
    print("   local-frame k_xyz",np.round([1/Cl[i,i] for i in range(3)],1),"k_rot",np.round([1/(Cl[i,i]*SPAN**2) for i in range(3,6)],1))
    wl=1/np.linalg.eigvalsh(Cm[:3,:3]).max(); wr=1/(np.linalg.eigvalsh(Cm[3:6,3:6]).max()*SPAN**2)
    print(name,"k_xyz",np.round(lin,1),"k_rot(xyz)",np.round(rot,1),"worst lin %.2f rot %.2f"%(wl,wr),flush=True)
mode=os.environ.get("FE_MODE","all")
Bring=np.unique(np.concatenate([ring_nodes(b) for b in Bb]))
Bseat=np.unique(np.concatenate([seat_nodes(zB)]+[ring_nodes(b) for b in Bb]))
for nm,fx in (("Arings",fixA_rings),):
    if mode in ("all","C"):
        report(nm+"+Brings-rigid",solve(fx,[(cB,Bring)]))
    if mode in ("all","S"):
        report(nm+"+Bseat-rigid",solve(fx,[(cB,Bseat)]))
    if mode in ("all","D"):
        bodies=[(np.array(b["position"]),ring_nodes(b)) for b in Bb]
        Cm=solve(fx,bodies)
        # per-bolt translational stiffness, rotations free
        for k,b in enumerate(Bb):
            c=Cm[6*k:6*k+3,6*k:6*k+3]
            print("  ",nm,"D",b["id"],"k_xyz",np.round(1/np.diag(c),1),"worst %.2f"%(1/np.linalg.eigvalsh(c).max()),flush=True)
        # load at B centre shared equally (forces) ; deflection = mean of bolt displacements
        nbt=len(Bb); 
        for i,ax in enumerate("xyz"):
            f=np.zeros(6*nbt); f[i::6]=1/nbt; u=Cm@f; print("   share F%s: mean disp %.4f -> k %.2f ; max bolt disp %.4f -> k %.2f"%(ax,u[i::6].mean(),1/u[i::6].mean(),np.abs(u[i::6]).max(),1/np.abs(u[i::6]).max()),flush=True)
    print("time",round(time.time()-t0,1),flush=True)
def rbe3(fixed_nodes, sets):
    isfix=np.zeros(len(ids),bool); isfix[fixed_nodes]=True
    free=np.where(~isfix)[0]; fd=(free[:,None]*3+np.arange(3)).ravel()
    lu=spl.splu(K[fd][:,fd].tocsc())
    pos=-np.ones(N,int); pos[fd]=np.arange(len(fd))
    out={}
    for nm,nd in sets.items():
        nd=np.array([n for n in nd if not isfix[n]])
        c=X[nd].mean(0); r=X[nd]-c; W=len(nd)
        Iw=np.zeros((3,3))
        for a in r: Iw+=np.dot(a,a)*np.eye(3)-np.outer(a,a)
        Ii=np.linalg.inv(Iw)
        Fm=np.zeros((len(fd),6))
        for j in range(6):
            Fv=np.zeros(3); Mv=np.zeros(3)
            if j<3: Fv[j]=1; Mv=np.cross(cB-c,Fv)
            else: Mv[j-3]=1
            f=Fv/W+np.cross(Ii@Mv,r)
            for i in range(3): Fm[pos[3*nd+i],j]+=f[:,i]
        U=lu.solve(Fm)
        Cm=np.zeros((6,6))
        for j in range(6):
            u=np.stack([U[pos[3*nd+i],j] for i in range(3)],1)
            Uc=u.mean(0); th=Ii@np.cross(r,u).sum(0)
            Cm[:3,j]=Uc+np.cross(th,cB-c); Cm[3:,j]=th
        out[nm]=Cm
    return out
if mode=="R":
    for nm,fx in (("Arings",fixA_rings),):
        res=rbe3(fx,{"Brings":Bring})
        for k,v in res.items(): report(nm+"+RBE3-"+k,v)
        print("time",round(time.time()-t0,1),flush=True)
if mode=="X":
    OA_=np.array(P["placements"]["A"]["origin"])
    def tolocal(p): return RA_.T@(p-OA_)
    XL=(X-OA_)@RA_      # local coords of nodes
    isfix=np.zeros(len(ids),bool); isfix[fixA_rings]=True
    free=np.where(~isfix)[0]; fd=(free[:,None]*3+np.arange(3)).ravel()
    lu=spl.splu(K[fd][:,fd].tocsc()); pos=-np.ones(N,int); pos[fd]=np.arange(len(fd))
    which=os.environ.get("FE_BOLT","B3"); dirl=np.array([float(v) for v in os.environ.get("FE_DIR","1,0,0").split(",")])
    b=[b for b in Bb if b["id"]==which][0]; nd=ring_nodes(b); nd=nd[~isfix[nd]]
    fw=RA_@dirl/len(nd); F=np.zeros(len(fd))
    for i in range(3): F[pos[3*nd+i]]+=fw[i]
    U=np.zeros(N); U[fd]=lu.solve(F); UL=U.reshape(-1,3)@RA_
    print("bolt",which,"dir",dirl,"mean disp local",np.round(UL[nd].mean(0),4))
    pts=json.loads(os.environ.get("FE_PTS"))
    for name,p in pts.items():
        d=np.linalg.norm(XL-np.array(p),axis=1); i=d.argmin()
        print("  %-22s at %s (d=%.1f): u_local = %s"%(name,np.round(XL[i],1),d[i],np.round(UL[i],4)))
