import sys, json, math, time
import numpy as np
import scipy.sparse as sp, scipy.sparse.linalg as spl
from build123d import *

H = float(sys.argv[2]) if len(sys.argv) > 2 else 1.2
E, NU = 2400.0, 0.35
task = json.load(open('/home/user/stepbench-runs/tasks/flange-L3-s1/task.json'))
F = {f['part']: f for f in task['poses'][0]['resolved']['mounting_faces']}
def fr(p):
    o = np.array(F[p]['origin']); n = np.array(F[p]['normal']); t = np.array([-n[1], n[0], 0.0]); t /= np.linalg.norm(t)
    return o, n, t
oA, nA, tA = fr('A'); oB, nB, tB = fr('B')
Rm = np.array([tA, nA, [0, 0, 1.0]])   # world -> frame A (u,n,z)
part = import_step(sys.argv[1])
verts, tris = part.tessellate(0.05, 0.2)
V = np.array([(v.X, v.Y, v.Z) for v in verts]); T = np.array(tris)
Vf = (V - oA) @ Rm.T       # coordinates (u,n,z-?) note z relative to oA.z
lo = Vf.min(0) - 2*H; hi = Vf.max(0) + 2*H
lo = np.floor(lo / H) * H
nx, ny, nz = np.ceil((hi - lo) / H).astype(int)
print('grid', nx, ny, nz)
# ray along z through cell centres
cx = lo[0] + (np.arange(nx) + 0.5) * H
cy = lo[1] + (np.arange(ny) + 0.5) * H
cross = {}
tri = Vf[T]   # (m,3,3)
count = 0
zs_list = [[] for _ in range(nx * ny)]
for k in range(len(tri)):
    a, b, c = tri[k]
    xmin, xmax = min(a[0], b[0], c[0]), max(a[0], b[0], c[0])
    ymin, ymax = min(a[1], b[1], c[1]), max(a[1], b[1], c[1])
    i0 = max(0, int(math.ceil((xmin - lo[0]) / H - 0.5))); i1 = min(nx - 1, int(math.floor((xmax - lo[0]) / H - 0.5)))
    j0 = max(0, int(math.ceil((ymin - lo[1]) / H - 0.5))); j1 = min(ny - 1, int(math.floor((ymax - lo[1]) / H - 0.5)))
    if i1 < i0 or j1 < j0: continue
    X, Y = np.meshgrid(cx[i0:i1 + 1], cy[j0:j1 + 1], indexing='ij')
    d = (b[1] - c[1]) * (a[0] - c[0]) + (c[0] - b[0]) * (a[1] - c[1])
    if abs(d) < 1e-12: continue
    l1 = ((b[1] - c[1]) * (X - c[0]) + (c[0] - b[0]) * (Y - c[1])) / d
    l2 = ((c[1] - a[1]) * (X - c[0]) + (a[0] - c[0]) * (Y - c[1])) / d
    l3 = 1 - l1 - l2
    m = (l1 >= 0) & (l2 >= 0) & (l3 >= 0)
    if not m.any(): continue
    Z = l1 * a[2] + l2 * b[2] + l3 * c[2]
    ii, jj = np.nonzero(m)
    for p_, q_ in zip(ii, jj):
        zs_list[(i0 + p_) * ny + (j0 + q_)].append(Z[p_, q_])
occ = np.zeros((nx, ny, nz), bool)
zc = lo[2] + (np.arange(nz) + 0.5) * H
for idx, zs in enumerate(zs_list):
    if len(zs) < 2: continue
    zs = sorted(zs)
    # merge near-duplicate hits (edges shared by triangles)
    cl = [zs[0]]
    for z in zs[1:]:
        if z - cl[-1] > 1e-6: cl.append(z)
    if len(cl) % 2: continue
    i, j = divmod(idx, ny)
    for p in range(0, len(cl), 2):
        occ[i, j, (zc > cl[p]) & (zc < cl[p + 1])] = True
print('elements', occ.sum(), 'volume', occ.sum() * H**3)
# nodes
nodeid = -np.ones((nx + 1, ny + 1, nz + 1), int)
ei, ej, ek = np.nonzero(occ)
corner = np.array([[0,0,0],[1,0,0],[1,1,0],[0,1,0],[0,0,1],[1,0,1],[1,1,1],[0,1,1]])
nodes_used = np.zeros((nx + 1, ny + 1, nz + 1), bool)
for c in corner: nodes_used[ei + c[0], ej + c[1], ek + c[2]] = True
nn = nodes_used.sum(); nodeid[nodes_used] = np.arange(nn)
print('nodes', nn)
ncoord = np.stack(np.nonzero(nodes_used), 1) * H + lo
conn = np.stack([nodeid[ei + c[0], ej + c[1], ek + c[2]] for c in corner], 1)
# element stiffness (hex8, 2x2x2 gauss, unit cube scaled H)
Dm = E / ((1 + NU) * (1 - 2 * NU)) * np.array([[1-NU, NU, NU, 0, 0, 0],[NU,1-NU,NU,0,0,0],[NU,NU,1-NU,0,0,0],[0,0,0,(1-2*NU)/2,0,0],[0,0,0,0,(1-2*NU)/2,0],[0,0,0,0,0,(1-2*NU)/2]])
g = 1 / math.sqrt(3); gp = [-g, g]
sgn = np.array([[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],[-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]])
Ke = np.zeros((24, 24))
for xi in gp:
    for et in gp:
        for ze in gp:
            dN = np.zeros((8, 3))
            for a in range(8):
                s = sgn[a]
                dN[a] = [s[0]*(1+s[1]*et)*(1+s[2]*ze)/8, s[1]*(1+s[0]*xi)*(1+s[2]*ze)/8, s[2]*(1+s[0]*xi)*(1+s[1]*et)/8]
            dN = dN * (2 / H)   # d/dx
            B = np.zeros((6, 24))
            for a in range(8):
                B[0, 3*a] = dN[a,0]; B[1, 3*a+1] = dN[a,1]; B[2, 3*a+2] = dN[a,2]
                B[3, 3*a] = dN[a,1]; B[3, 3*a+1] = dN[a,0]
                B[4, 3*a+1] = dN[a,2]; B[4, 3*a+2] = dN[a,1]
                B[5, 3*a] = dN[a,2]; B[5, 3*a+2] = dN[a,0]
            Ke += B.T @ Dm @ B * (H / 2) ** 3
dofs = (conn[:, :, None] * 3 + np.arange(3)).reshape(len(conn), 24)
rows = np.repeat(dofs, 24, axis=1).ravel(); cols = np.tile(dofs, (1, 24)).ravel()
K = sp.coo_matrix((np.tile(Ke.ravel(), len(conn)), (rows, cols)), shape=(3*nn, 3*nn)).tocsr()
# boundary sets, nodes in frame-A coords: x=u, y=n, z=world z (relative oA)
Wn = ncoord  # (u, n, zrel)
world = Wn @ Rm + oA  # inverse rotation: row vec @ Rm
# A seat: nodes with n < tol and r inside disc 37 (contact region)
tolA = 0.6 * H
fixA = (np.abs(Wn[:, 1]) < tolA)
import os
PADR = float(os.environ.get('PADR', '0'))
bolts = task['poses'][0]['resolved']['bolts']
def near_bolts(world, part, R):
    m = np.zeros(len(world), bool)
    for b in bolts:
        if b['id'][0] != part: continue
        p = np.array(b['position']); ax = np.array(b['axis'])
        d = world - p
        along = d @ ax
        rad = np.linalg.norm(d - np.outer(along, ax), axis=1)
        m |= rad < R
    return m

# B seat: signed distance to B plane
dB = (world - oB) @ nB
uB = (world - oB) @ tB
wB = world[:, 2] - oB[2]
selB = (np.abs(dB) < 0.6 * H) & (np.abs(uB) < 24) & (np.abs(wB) < 49)
# bolt hole nodes also fixed on A (seat + holes): nodes within hole walls near plane are included by tol only; fine
MODE = os.environ.get('MODE', 'both')
if PADR > 0:
    if MODE in ('both', 'A'): fixA &= near_bolts(world, 'A', PADR)
    if MODE in ('both', 'B'):
        selB &= near_bolts(world, 'B', PADR)
        if os.environ.get('HOLEWALL'):
            hw = near_bolts(world, 'B', 3.8) & (dB > -0.9) & (dB < 8.0)
            selB |= hw
print('A-fixed nodes', fixA.sum(), 'B nodes', selB.sum())
# master node at B centre
cB = oB
rel = world[selB] - cB
Sidx = np.nonzero(selB)[0]
nS = len(Sidx)
R = np.zeros((3 * nS, 6))
for q in range(nS):
    x, y, z = rel[q]
    R[3*q:3*q+3, 0:3] = np.eye(3)
    R[3*q:3*q+3, 3:6] = np.array([[0, z, -y], [-z, 0, x], [y, -x, 0]])
ndof = 3 * nn
isfix = np.zeros(ndof, bool)
for d in range(3): isfix[np.nonzero(fixA)[0] * 3 + d] = True
Sd = (Sidx[:, None] * 3 + np.arange(3)).ravel()
isS = np.zeros(ndof, bool); isS[Sd] = True
free = np.nonzero(~isfix & ~isS)[0]
# reduced system
Kff = K[free][:, free]; KfS = K[free][:, Sd]; KSS = K[Sd][:, Sd]
Rs = sp.csr_matrix(R)
Kfm = KfS @ Rs
Kmm = (Rs.T @ KSS @ Rs).toarray()
nf = len(free)
Kfull = sp.bmat([[Kff, Kfm], [Kfm.T, sp.csr_matrix(Kmm)]], format='csc')
t0 = time.time()
lu = spl.splu(Kfull)
print('factor', time.time() - t0)
span = 74.54; Fl = 7.85
res = {}
# directions in world: x,y,z; use B frame axes too
dirs = {'tB': tB, 'nB': nB, 'z': np.array([0, 0, 1.0])}
for name, d in dirs.items():
    rhs = np.zeros(nf + 6); rhs[nf:nf + 3] = Fl * d
    x = lu.solve(rhs); qm = x[nf:]
    res['F_' + name] = Fl / np.linalg.norm(qm[:3])
    rhs = np.zeros(nf + 6); rhs[nf + 3:nf + 6] = Fl * span * d
    x = lu.solve(rhs); qm = x[nf:]
    res['M_' + name] = Fl / (np.linalg.norm(qm[3:]) * span)
print({k: round(v, 2) for k, v in res.items()})
print('min stiffness', min(res.values()))

# ---- energy distribution for the weakest case
if os.environ.get('ENERGY'):
    case = os.environ['ENERGY']
    d = dirs[case[2:]]
    rhs = np.zeros(nf + 6)
    if case[0] == 'M': rhs[nf + 3:nf + 6] = Fl * span * d
    else: rhs[nf:nf + 3] = Fl * d
    x = lu.solve(rhs)
    u = np.zeros(ndof); u[free] = x[:nf]; u[Sd] = R @ x[nf:]
    ue = u[dofs]
    en = 0.5 * np.einsum('ij,jk,ik->i', ue, Ke, ue)
    cen = ncoord[conn].mean(1)
    cw = cen @ Rm + oA
    nBc = (cw - oB) @ nB
    wz = cw[:, 2]
    print('total energy', en.sum())
    for lo_, hi_ in [(-5, 5), (5, 15), (15, 30), (30, 50), (50, 75), (75, 100)]:
        m = (nBc >= lo_) & (nBc < hi_)
        print('nB', lo_, hi_, round(en[m].sum() / en.sum(), 3))
    for lo_, hi_ in [(-70, -40), (-40, -20), (-20, 20), (20, 40)]:
        m = (wz >= lo_) & (wz < hi_) & (nBc < 8)
        print('B plate z', lo_, hi_, round(en[m].sum() / en.sum(), 3))

    uB_c = (cw - oB) @ tB
    print('--- energy by (|u| bin, w bin) near B plate (nB<16)')
    for ub in [(0,6),(6,11),(11,19),(19,30),(30,80)]:
        row=[]
        for wb in [(-70,-48),(-48,-34),(-34,-10),(-10,12),(12,28),(28,44),(44,60)]:
            m=(nBc<16)&(np.abs(uB_c)>=ub[0])&(np.abs(uB_c)<ub[1])&((wz+18)>=wb[0]+0)&((wz+18)<wb[1])
            row.append(round(en[m].sum()/en.sum(),3))
        print(ub,row)
    m=(nBc>=16); print('rest beyond nB16', round(en[m].sum()/en.sum(),3))
