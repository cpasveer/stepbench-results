import sys, json, subprocess, os, time
import numpy as np
import scipy.sparse as sp
import scipy.sparse.linalg as spl
HERE=os.path.dirname(os.path.dirname(os.path.abspath(__file__))) if '__file__' in globals() else os.getcwd()
HERE='/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1'
T=json.load(open(HERE+'/task.json'))
step=os.environ.get('STEP',HERE+'/output.step')
cl=float(os.environ.get('CL','3.0'))
E=float(os.environ.get('YM','3500'))
nu=0.36
msh=HERE+'/scratch/fea_%d.msh'%os.getpid()
t0=time.time()
subprocess.run(['gmsh','-3','-order','2','-clmax',str(cl),'-format','msh2','-o',msh,step],stdout=subprocess.DEVNULL,stderr=subprocess.DEVNULL,check=True)
# parse
L=open(msh).read().split('\n')
i=L.index('$Nodes');nn=int(L[i+1]);X=np.zeros((nn+1,3))
ids=[]
for k in range(nn):
    a=L[i+2+k].split();X[int(a[0])]=[float(a[1]),float(a[2]),float(a[3])]
i=L.index('$Elements');ne=int(L[i+1]);tets=[]
for k in range(ne):
    a=L[i+2+k].split()
    if a[1]=='11':
        nt=int(a[2]);tets.append([int(v) for v in a[3+nt:3+nt+10]])
tets=np.array(tets)
used=np.unique(tets);remap=-np.ones(nn+1,int);remap[used]=np.arange(len(used))
Xn=X[used];tets=remap[tets];N=len(used)
print('nodes',N,'tets',len(tets),'mesh s',round(time.time()-t0,1))
# shape function derivs wrt barycentric
def shape(L):
    l0,l1,l2,l3=L;Ls=[l0,l1,l2,l3]
    N=[l*(2*l-1) for l in Ls]
    ed=[(0,1),(1,2),(2,0),(3,0),(3,2),(3,1)]
    N+= [4*Ls[a]*Ls[b] for a,b in ed]
    return N
def dshape_dxi(L):
    # derivative wrt xi=L1, eta=L2, zeta=L3 (L0=1-sum)
    l=L
    dL=np.array([[-1,-1,-1],[1,0,0],[0,1,0],[0,0,1]],float)
    d=np.zeros((10,3))
    for i in range(4): d[i]=(4*l[i]-1)*dL[i]
    ed=[(0,1),(1,2),(2,0),(3,0),(3,2),(3,1)]
    for k,(a,b) in enumerate(ed): d[4+k]=4*(l[a]*dL[b]+l[b]*dL[a])
    return d
a=0.5854101966249685;b=0.1381966011250105
gps=[(a,b,b,b),(b,a,b,b),(b,b,a,b),(b,b,b,a)]
def iso(E,nu):
    return E/((1+nu)*(1-2*nu))*np.array([[1-nu,nu,nu,0,0,0],[nu,1-nu,nu,0,0,0],[nu,nu,1-nu,0,0,0],[0,0,0,(1-2*nu)/2,0,0],[0,0,0,0,(1-2*nu)/2,0],[0,0,0,0,0,(1-2*nu)/2]])
EZ=float(os.environ.get('EZ','0'))
if EZ>0:
    Ep=E;nup=0.36;nuzp=float(os.environ.get('NUZP','0.3'))
    Gp=Ep/(2*(1+nup))*float(os.environ.get('GPF','1'));Gz=EZ/(2*(1+nuzp))*float(os.environ.get('GZF','1'));nupz=nuzp*Ep/EZ
    S=np.zeros((6,6))
    S[0,0]=S[1,1]=1/Ep;S[2,2]=1/EZ
    S[0,1]=S[1,0]=-nup/Ep
    S[0,2]=S[2,0]=S[1,2]=S[2,1]=-nuzp/EZ
    S[3,3]=S[4,4]=1/Gz;S[5,5]=1/Gp
    Dl=np.linalg.inv(S)
    ud=np.array([float(x) for x in os.environ.get('UDIR','0.3885173,0.9214414,0').split(',')]);ud/=np.linalg.norm(ud)
    tmp=np.array([1.,0,0]) if abs(ud[0])<0.9 else np.array([0,1.,0])
    ex=np.cross(tmp,ud);ex/=np.linalg.norm(ex);ey=np.cross(ud,ex)
    Rr=np.array([ex,ey,ud])  # rows: local axes in global
    vm=[(0,0),(1,1),(2,2),(1,2),(0,2),(0,1)]
    Cl=np.zeros((3,3,3,3))
    for I,(i,j) in enumerate(vm):
        for J,(k,l) in enumerate(vm):
            for (a,b) in {(i,j),(j,i)}:
                for (c,d) in {(k,l),(l,k)}:
                    Cl[a,b,c,d]=Dl[I,J]
    Cg=np.einsum('ai,bj,ck,dl,abcd->ijkl',Rr,Rr,Rr,Rr,Cl)
    D=np.array([[Cg[i,j,k,l] for (k,l) in vm] for (i,j) in vm])
else:
    D=iso(E,nu)
xe=Xn[tets]  # ne,10,3
ESC=np.ones(len(tets))
SK=float(os.environ.get('SKIN','0'))
if SK>0:
    import FreeCAD as App, Part
    shp=Part.read(step)
    cent=xe[:,:4,:].mean(axis=1)
    for i in range(len(tets)):
        d=Part.Vertex(App.Vector(*cent[i])).distToShape(shp.Shells[0])[0] if False else shp.Shells[0].distToShape(Part.Vertex(App.Vector(*cent[i])))[0]
        ESC[i]=1.0 if d<=SK else float(os.environ.get('EINF','0.2'))
    print('skin elements',(ESC==1).sum(),'of',len(ESC))
Ke=np.zeros((len(tets),30,30))
for g in gps:
    dN=dshape_dxi(g)  # 10,3
    J=np.einsum('na,eni->eai',dN,xe) if False else np.einsum('ni,enj->eij',dN,xe)  # J[e,i,j]=sum_n dN[n,i]*x[e,n,j]
    detJ=np.linalg.det(J);invJ=np.linalg.inv(J)
    dNx=np.einsum('eik,nk->eni',invJ,dN)  # wait need dN/dx_i = sum_k invJ[i,k] dN/dxi_k
    B=np.zeros((len(tets),6,30))
    for n in range(10):
        B[:,0,3*n]=dNx[:,n,0];B[:,1,3*n+1]=dNx[:,n,1];B[:,2,3*n+2]=dNx[:,n,2]
        B[:,3,3*n]=dNx[:,n,1];B[:,3,3*n+1]=dNx[:,n,0]
        B[:,4,3*n+1]=dNx[:,n,2];B[:,4,3*n+2]=dNx[:,n,1]
        B[:,5,3*n]=dNx[:,n,2];B[:,5,3*n+2]=dNx[:,n,0]
    Ke+=np.einsum('eki,kl,elj,e->eij',B,D,B,np.abs(detJ)/6.0/1.0*1.0*(1.0/1.0)*0.25*6.0/6.0*1.0) if False else np.einsum('eki,kl,elj,e->eij',B,D,B,np.abs(detJ)*(1/6.)*0.25*6/6*1.0*ESC)
# NOTE: weight: integral over tet = detJ * (1/6) volume of ref tet; 4 pts each weight 1/4 of that  => detJ/24
# (above multiplier: detJ*(1/6)*0.25*1 = detJ/24)
dof=(3*tets[:,:,None]+np.arange(3)[None,None,:]).reshape(len(tets),30)
rows=np.repeat(dof[:,:,None],30,axis=2).ravel();cols=np.repeat(dof[:,None,:],30,axis=1).ravel()
K=sp.coo_matrix((Ke.ravel(),(rows,cols)),shape=(3*N,3*N)).tocsr()
print('assembled',round(time.time()-t0,1))
# BC
pose=T['poses'][0]
R=np.array(pose['placements']['A']['rotation']);c=np.array(pose['placements']['A']['origin']);ax=R[:,0]
rad=lambda p:np.linalg.norm((p-c)-np.outer((p-c)@ax,ax),axis=1)
bolts={b['id']:b for b in pose['resolved']['bolts']}
held=rad(Xn)<34.03
XCUT=os.environ.get('XCUT')
if XCUT:
    xc=float(XCUT);thc=float(os.environ.get('THC','30'))
    loc=Xn-c;xl=loc@ax
    # theta around top
    TOPv=R[:,2];SIDEv=np.cross(ax,TOPv)
    th=np.degrees(np.arctan2(loc@SIDEv,loc@TOPv))
    excl=(xl>xc)&(np.abs(th)<thc)
    held&=~excl
    print('excluded held nodes',excl.sum())
HR=float(os.environ.get('HR','0'))
if HR>0:
    near=np.zeros(len(Xn),bool)
    for k,b in bolts.items():
        if k[0]!='A':continue
        p=np.array(b['position']);d=np.array(b['axis']);r=Xn-p;al=r@d;rr=np.linalg.norm(r-np.outer(al,d),axis=1)
        near|=(rr<HR)
    held&=near
for k,b in bolts.items():
    if k[0]!='A':continue
    p=np.array(b['position']);d=np.array(b['axis']);r=Xn-p;al=r@d;rr=np.linalg.norm(r-np.outer(al,d),axis=1)
    held|=(rr<2.75+0.05)&(al<4.2)&(al>-1)
face=pose['resolved']['mounting_faces'][0];Bc=np.array(face['origin']);nB=np.array(face['normal'])
bn=((Xn-Bc)@nB)<0.03
RB=float(os.environ.get('RB','0'))
if RB>0:
    nearB=np.zeros(len(Xn),bool)
    for k,b in bolts.items():
        if k[0]!='B':continue
        p=np.array(b['position']);d=np.array(b['axis']);r=Xn-p;al=r@d;rr=np.linalg.norm(r-np.outer(al,d),axis=1)
        nearB|=(rr<RB)
    bn&=nearB
for k,b in bolts.items():
    if k[0]!='B':continue
    p=np.array(b['position']);d=np.array(b['axis']);r=Xn-p;al=r@d;rr=np.linalg.norm(r-np.outer(al,d),axis=1)
    bn|=(rr<2.25+0.05)&(al<3.7)&(al>-1)
bn&=~held
print('held',held.sum(),'Bnodes',bn.sum())
free=~(held|bn)
# reduced space: free node dofs + 6 master
fidx=np.where(free)[0];bidx=np.where(bn)[0]
nf=len(fidx)
rowsT=[];colsT=[];valsT=[]
# free dofs
fd=(3*fidx[:,None]+np.arange(3)).ravel()
rowsT+=list(fd);colsT+=list(range(3*nf));valsT+=[1.0]*(3*nf)
r=Xn[bidx]-Bc
for j,node in enumerate(bidx):
    for ci in range(3):
        row=3*node+ci
        rowsT.append(row);colsT.append(3*nf+ci);valsT.append(1.0)
        # theta x r : (th_y*r_z - th_z*r_y, th_z*r_x - th_x*r_z, th_x*r_y - th_y*r_x)
        rx,ry,rz=r[j]
        if ci==0: rowsT+= [row,row];colsT+=[3*nf+4,3*nf+5];valsT+=[rz,-ry]
        if ci==1: rowsT+= [row,row];colsT+=[3*nf+5,3*nf+3];valsT+=[rx,-rz]
        if ci==2: rowsT+= [row,row];colsT+=[3*nf+3,3*nf+4];valsT+=[ry,-rx]
Tm=sp.coo_matrix((valsT,(rowsT,colsT)),shape=(3*N,3*nf+6)).tocsr()
Kr=(Tm.T@K@Tm).tocsc()
lu=spl.splu(Kr[:3*nf,:3*nf]) if False else None
# solve for 6 unit loads on master: K_red u = f. Partition free(ff), master(mm)
Kff=Kr[:3*nf,:3*nf];Kfm=Kr[:3*nf,3*nf:];Kmm=Kr[3*nf:,3*nf:].toarray()
lu=spl.splu(Kff.tocsc())
Z=lu.solve(Kfm.toarray())
S=Kmm-Kfm.T@Z   # 6x6 stiffness of rigid washer
Sx=(S+S.T)/2
print('K6 diag',np.diag(Sx).round(1))
Ls=78.15
Dm=np.diag([1,1,1,Ls,Ls,Ls]);Di=np.linalg.inv(Dm)
Kd=Di@Sx@Di
w=np.linalg.eigvalsh(Kd)
print('eig K_delta',w.round(2))
print('trans min',np.linalg.eigvalsh(Sx[:3,:3]).min().round(2),'rot min (/L^2)',(np.linalg.eigvalsh(Sx[3:,3:]).min()/Ls**2).round(2))
print('time',round(time.time()-t0,1))
ea=np.array([0,0,1.]);eb=np.cross(nB,ea)
Rm=np.array([ea,eb,nB])  # rows
w,v=np.linalg.eigh(Kd)
for i in range(3):
    vi=v[:,i]
    print('mode',round(w[i],2),'force(ea,eb,n)',(Rm@vi[:3]).round(2),'moment/L (ea,eb,n)',(Rm@vi[3:]).round(2))
# directional stiffness table
for nm,dv in (('ea',ea),('eb',eb),('n',nB)):
    f=np.zeros(6);f[:3]=dv;print('k force',nm,round(1/(f@np.linalg.solve(Kd,f)),2))
    f=np.zeros(6);f[3:]=dv;print('k moment',nm,round(1/(f@np.linalg.solve(Kd,f)),2))
# alternative metric: force d applied at lever point L*e (e perp d); displacement along d at that point
C=np.linalg.inv(Kd)
rng=np.random.default_rng(0)
best=1e9;bestk=None
for _ in range(20000):
    d=rng.normal(size=3);d/=np.linalg.norm(d)
    e=rng.normal(size=3);e-=e@d*d;e/=np.linalg.norm(e)
    g=np.concatenate([d,np.cross(e,d)])
    k=1/(g@C@g)
    if k<best:best=k;bestk=(d,e)
print('lever metric min',round(best,2),'d(ea,eb,n)',(Rm@bestk[0]).round(2),'e',(Rm@bestk[1]).round(2))
# pure torque as force couple?  min over axes of torque stiffness /L^2 handled above
print('--- frames')
Cmat=np.linalg.inv(Kd)
fr={'world':np.eye(3),'B':Rm,'pipe':np.array([ax,R[:,2],np.cross(ax,R[:,2])])}
for nm,F in fr.items():
    out=[]
    for i in range(3):
        f=np.zeros(6);f[:3]=F[i];out.append(round(1/(f@Cmat@f),1))
    for i in range(3):
        f=np.zeros(6);f[3:]=F[i];out.append(round(1/(f@Cmat@f),1))
    print(nm,'force',out[:3],'moment',out[3:])
# translation-only compliance (rotation free) and rotation-only
Ct=np.linalg.inv(Sx[:3,:3]);print('pure-force free-rot... translation block min k',round(1/np.linalg.eigvalsh(np.linalg.inv(Sx[:3,:3])).max(),1))
np.save(os.environ.get('SAVE','scratch/S.npy'),Sx)
os.remove(msh)
