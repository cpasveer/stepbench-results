import numpy as np, json, sys
T = json.load(open("task.json"))
P0 = np.array([21.0, 136.5, -7.0]); AX = np.array([0.2487956429525063, 0.5900653629615055, 0.7680648380709507])
CB = np.array(T["poses"][0]["placements"]["B"]["origin"])
NB = np.array(T["poses"][0]["resolved"]["mounting_faces"][0]["normal"])
UB = np.array([0.9214414224885092, -0.3885173161164798, 0.0]); ZB = np.array([0,0,1.0])
A = {b["id"]: np.array(b["axis"]) for b in T["poses"][0]["resolved"]["bolts"]}
def beam_k(p, q, E, G, Aa, I1, I2, J):
    """12x12 local-to-global Euler-Bernoulli frame element stiffness."""
    L = np.linalg.norm(q-p); x = (q-p)/L
    ref = np.array([0,0,1.0]) if abs(x[2])<0.9 else np.array([1.0,0,0])
    y = np.cross(ref, x); y/=np.linalg.norm(y); z = np.cross(x, y)
    k = np.zeros((12,12))
    EA=E*Aa/L; GJ=G*J/L
    k[0,0]=k[6,6]=EA; k[0,6]=k[6,0]=-EA
    k[3,3]=k[9,9]=GJ; k[3,9]=k[9,3]=-GJ
    # bending in local xy plane (about z): uses I_z
    a1=12*E*I2/L**3; a2=6*E*I2/L**2; a3=4*E*I2/L; a4=2*E*I2/L
    idx=[1,5,7,11]
    kb=np.array([[a1,a2,-a1,a2],[a2,a3,-a2,a4],[-a1,-a2,a1,-a2],[a2,a4,-a2,a3]])
    for i,ii in enumerate([1,5,7,11]):
        for j,jj in enumerate([1,5,7,11]): k[ii,jj]=kb[i,j]
    b1=12*E*I1/L**3; b2=6*E*I1/L**2; b3=4*E*I1/L; b4=2*E*I1/L
    kz=np.array([[b1,-b2,-b1,-b2],[-b2,b3,b2,b4],[-b1,b2,b1,b2],[-b2,b4,b2,b3]])
    for i,ii in enumerate([2,4,8,10]):
        for j,jj in enumerate([2,4,8,10]): k[ii,jj]=kz[i,j]
    R = np.zeros((12,12))
    M = np.vstack([x,y,z])
    for blk in range(4): R[3*blk:3*blk+3,3*blk:3*blk+3]=M
    return R.T @ k @ R
def solve(nodes, members, fixed, load_node, E, G):
    """nodes: list of xyz; members: (i,j,A,I,J); fixed: set of node ids; load at node (force vector F)."""
    n=len(nodes); K=np.zeros((6*n,6*n))
    for (i,j,Aa,I,J) in members:
        ke=beam_k(nodes[i],nodes[j],E,G,Aa,I,I,J)
        dof=list(range(6*i,6*i+6))+list(range(6*j,6*j+6))
        K[np.ix_(dof,dof)]+=ke
    free=[d for d in range(6*n) if d//6 not in fixed]
    Kff=K[np.ix_(free,free)]
    out={}
    for d in range(3):
        F=np.zeros(6*n); F[6*load_node+d]=1.0
        u=np.linalg.solve(Kff, F[free])
        out[d]=u[[free.index(6*load_node+d)]][0]
    return out
