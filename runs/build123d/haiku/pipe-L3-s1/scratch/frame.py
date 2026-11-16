import numpy as np
def rot_frame(a):
    a = np.asarray(a, float); a = a/np.linalg.norm(a)
    t = np.array([1.0,0,0]) if abs(a[0])<0.9 else np.array([0,1.0,0])
    e1 = np.cross(a,t); e1/=np.linalg.norm(e1); e2=np.cross(a,e1)
    return a,e1,e2
def beam_k(p1,p2,r,E=2000.0,nu=0.38):
    """6x6-node local stiffness, global, 12x12 matrix for circular beam"""
    p1=np.asarray(p1,float); p2=np.asarray(p2,float)
    L=np.linalg.norm(p2-p1); x=(p2-p1)/L
    A=np.pi*r*r; I=np.pi*r**4/4; J=2*I; G=E/(2*(1+nu))
    k=np.zeros((12,12))
    EA=E*A/L; GJ=G*J/L
    EI1=E*I/L**3; EI2=E*I/L**3
    k[0,0]=k[6,6]=EA; k[0,6]=k[6,0]=-EA
    k[3,3]=k[9,9]=GJ; k[3,9]=k[9,3]=-GJ
    # bending in local y (uses v,theta_z) and local z (w,theta_y)
    kb=np.array([[12,6*L,-12,6*L],[6*L,4*L*L,-6*L,2*L*L],[-12,-6*L,12,-6*L],[6*L,2*L*L,-6*L,4*L*L]])*E*I/L**3
    idx_y=[1,5,7,11]; idx_z=[2,4,8,10]
    # local y-direction: v at 1, rz at 5 ; v at 7, rz at 11 ; sign for rz
    sy=[1,1,1,1]
    for a_,ia in enumerate(idx_y):
        for b_,ib in enumerate(idx_y):
            k[ia,ib]+=kb[a_,b_]
    kz=np.array([[12,-6*L,-12,-6*L],[-6*L,4*L*L,6*L,2*L*L],[-12,6*L,12,6*L],[-6*L,2*L*L,6*L,4*L*L]])*E*I/L**3
    idx_z=[2,4,8,10]
    for a_,ia in enumerate(idx_z):
        for b_,ib in enumerate(idx_z):
            k[ia,ib]+=kz[a_,b_]
    # transform local->global: local frame: x along member, y=e1, z=e2
    a,e1,e2=rot_frame(x)
    T3=np.vstack([x,e1,e2])
    R=np.zeros((12,12))
    for i in range(4): R[3*i:3*i+3,3*i:3*i+3]=T3
    return R.T@k@R
def solve(nodes, members, fixed, loads, E=2000.0):
    """nodes: (N,3); members: list of (i,j,r); fixed: list of node idx (all dof fixed);
    loads: dict node-> 6-vector. returns displacement array (N,6)"""
    N=len(nodes); K=np.zeros((6*N,6*N))
    for i,j,r in members:
        ke=beam_k(nodes[i],nodes[j],r,E)
        dofs=list(range(6*i,6*i+6))+list(range(6*j,6*j+6))
        K[np.ix_(dofs,dofs)]+=ke
    F=np.zeros(6*N)
    for n,v in loads.items(): F[6*n:6*n+6]+=np.asarray(v,float)
    free=[d for d in range(6*N) if not any(d//6==f for f in fixed)]
    # tiny regularisation for rotational dofs of unconnected nodes
    Kf=K[np.ix_(free,free)]+np.eye(len(free))*1e-9
    u=np.zeros(6*N)
    u[free]=np.linalg.solve(Kf,F[free])
    return u.reshape(N,6)
