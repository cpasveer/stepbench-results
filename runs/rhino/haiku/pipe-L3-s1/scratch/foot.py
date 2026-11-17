import math
from frame import nrm, sub, dot, cross, transform, build_T, mat_mul, T_t, solve, frame_k
EE=2488.0; GG=EE/2.6
C=(19.549595153015062,49.68566626427389,52.2472790015625); N=(0.3885173161164798,0.9214414224885092,0.0)
U=(0.0,0.0,1.0); V=(0.9214414224885092,-0.3885173161164798,0.0)
A1=(0.7093118905965579,-0.6509870200868549,0.27035632165831364); A3=(-0.3771547728198181,-0.6713858946506742,0.6379617989560074)
m=[A1[i]+A3[i] for i in range(3)]; l=nrm(m); m=[x/l for x in m]
D=(0.2487956429525063,0.5900653629615055,0.7680648380709507)
P0=(21.0,136.5,-7.0)
t=cross(D,m); tn=nrm(t); t=[x/tn for x in t]
E2=[P0[i]+36*m[i]-6*D[i]+8*t[i] for i in range(3)]
R0=[C[i]-6.0*U[i]+2.0*V[i] for i in range(3)]
pads=[(0.7093,-0.6510,0.2704)]
import json
TASK=json.load(open('./task.json'))
B={b['id']:b for b in TASK['poses'][0]['resolved']['bolts']}
PADS=[tuple(B[i]['position']) for i in ['A1','A2','A3','A4']]
PADS=[tuple(p) for p in PADS]
def beam(nodes_pairs, sections):
    pass
def solve_frame(elements, ndof_nodes, fixed_nodes, load_node, load_dir):
    # elements: list of (node_i, node_j, xyz_i, xyz_j, A, Iy, Iz, J)
    ndof=6*ndof_nodes
    K=[[0.0]*ndof for _ in range(ndof)]
    for (ni,nj,pi,pj,A,Iy,Iz,J) in elements:
        L=nrm(sub(pj,pi))
        xa=[x/L for x in sub(pj,pi)]
        ref=[0,0,1] if abs(xa[2])<0.9 else [0,1,0]
        R=transform(xa, ref)
        Tm=[[0.0]*12 for _ in range(12)]
        for blk in range(4):
            for a in range(3):
                for b in range(3):
                    Tm[blk*3+a][blk*3+b]=R[a][b]
        kl=frame_k(EE,GG,A,Iy,Iz,J,L)
        kg=mat_mul(T_t(Tm),mat_mul(kl,Tm))
        dofs=list(range(6*ni,6*ni+6))+list(range(6*nj,6*nj+6))
        for a in range(12):
            for b in range(12):
                K[dofs[a]][dofs[b]]+=kg[a][b]
    free=[d for d in range(ndof) if (d//6) not in fixed_nodes]
    Kf=[[K[i][j] for j in free] for i in free]
    f=[0.0]*len(free)
    for k in range(3):
        if 6*load_node+k in free:
            f[free.index(6*load_node+k)]=load_dir[k]
    u=solve(Kf,f)
    disp=[0.0]*ndof
    for idx,d in enumerate(free): disp[d]=u[idx]
    return disp
def k_at(arm_a, arm_t, sp_w, sp_h, sp_top_offset=0.0, ntop=None):
    # nodes: 0 = R0 (load), 1 = E2 (arm top), 2.. = pads (fixed)
    Ai=arm_a*arm_a-(arm_a-2*arm_t)**2
    Ia=(arm_a**4-(arm_a-2*arm_t)**4)/12.0
    Ja=2*arm_t*(arm_a-arm_t)**2
    els=[(0,1,R0,E2,Ai,Ia,Ia,Ja)]
    nodes=[R0,E2]
    for p in PADS:
        # spoke from E2 to pad position (pad is fixed node)
        A=sp_w*sp_h; Iy=sp_w*sp_h**3/12; Iz=sp_h*sp_w**3/12; J=sp_w*sp_h**3/3*(1-0.63*sp_h/sp_w) if sp_w>sp_h else sp_h*sp_w**3/3*(1-0.63*sp_w/sp_h)
        nodes.append(p)
        els.append((1,len(nodes)-1,E2,p,A,Iy,Iz,J))
    fixed=set(range(2,len(nodes)))
    best=None
    for ld in ([1,0,0],[0,1,0],[0,0,1]):
        disp=solve_frame(els,len(nodes),fixed,0,ld)
        d=disp[0]*ld[0]+disp[1]*ld[1]+disp[2]*ld[2]
        k=1.0/d if d!=0 else 1e9
        best=k if best is None else min(best,k)
    return best
if __name__=="__main__":
    print("arm only (fix top):", "n/a")
    for sw,sh in [(5,3.5),(5,5),(6,5),(8,5),(8,6)]:
        print("spoke %g x %g  -> k = %.1f N/mm" % (sw,sh,k_at(12,1.6,sw,sh)))
