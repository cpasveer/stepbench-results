"""Rough 3D frame model: beams between nodes, A fixed, B rigid washer loaded.
Used only for sizing (not the checker). Units mm, N, MPa."""
import numpy as np

def beam_k(p1, p2, E, G, A, Iy, Iz, J):
    L = np.linalg.norm(p2 - p1)
    ex = (p2 - p1) / L
    tmp = np.array([0, 0, 1.0]) if abs(ex[2]) < 0.9 else np.array([0, 1.0, 0])
    ey = np.cross(tmp, ex); ey /= np.linalg.norm(ey)
    ez = np.cross(ex, ey)
    R = np.vstack([ex, ey, ez])
    k = np.zeros((12, 12))
    EA, GJ = E*A/L, G*J/L
    k[0,0]=k[6,6]=EA; k[0,6]=k[6,0]=-EA
    k[3,3]=k[9,9]=GJ; k[3,9]=k[9,3]=-GJ
    a1, b1 = 12*E*Iz/L**3, 6*E*Iz/L**2
    c1, d1 = 4*E*Iz/L, 2*E*Iz/L
    k[1,1]=k[7,7]=a1; k[1,7]=k[7,1]=-a1
    k[1,5]=k[5,1]=b1; k[1,11]=k[11,1]=b1
    k[5,7]=k[7,5]=-b1; k[7,11]=k[11,7]=-b1
    k[5,5]=k[11,11]=c1; k[5,11]=k[11,5]=d1
    a2, b2 = 12*E*Iy/L**3, 6*E*Iy/L**2
    c2, d2 = 4*E*Iy/L, 2*E*Iy/L
    k[2,2]=k[8,8]=a2; k[2,8]=k[8,2]=-a2
    k[2,4]=k[4,2]=-b2; k[2,10]=k[10,2]=-b2
    k[4,8]=k[8,4]=b2; k[8,10]=k[10,8]=b2
    k[4,4]=k[10,10]=c2; k[4,10]=k[10,4]=d2
    T = np.zeros((12,12))
    for i in range(4): T[3*i:3*i+3,3*i:3*i+3] = R
    return T.T @ k @ T, L

def solve(nodes, elems, fixed, loadnode, rigid_nodes):
    """nodes: dict name->xyz; elems: list of (n1,n2,props); fixed: list of names (all dof 0)
    loadnode: name of node carrying the B washer; rigid_nodes: names attached rigidly to loadnode."""
    names = list(nodes); idx = {n:i for i,n in enumerate(names)}
    N = 6*len(names)
    K = np.zeros((N,N))
    for n1,n2,pr in elems:
        ke,_ = beam_k(np.array(nodes[n1],float), np.array(nodes[n2],float), *pr)
        dofs = list(range(6*idx[n1],6*idx[n1]+6)) + list(range(6*idx[n2],6*idx[n2]+6))
        K[np.ix_(dofs,dofs)] += ke
    free = [i for i in range(N) if not any(i//6 == idx[f] for f in fixed)]
    Kf = K[np.ix_(free,free)]
    res = {}
    # compliance at loadnode: apply unit forces/moments on its 6 dofs
    li = 6*idx[loadnode]
    Ff = np.zeros((len(free),6))
    pos = {d:j for j,d in enumerate(free)}
    for j in range(6):
        Ff[pos[li+j],j] = 1.0
    U = np.linalg.solve(Kf, Ff)
    C = U[[pos[li+j] for j in range(6)],:]   # 6x6 compliance at loadnode
    return C

if __name__ == "__main__":
    pass
