"""Rough voxel FE (Wilson incompatible-mode hexes) to estimate the B-mount stiffness.
usage: python fe.py <step> [h] ; A held at bolt seat rings + hole walls, B bolts tied rigid."""
import sys, json, pathlib, math, time
import numpy as np, scipy.sparse as sp, scipy.sparse.linalg as spl
from build123d import import_step
from OCP.IntCurvesFace import IntCurvesFace_ShapeIntersector
from OCP.gp import gp_Lin, gp_Pnt, gp_Dir
HERE = pathlib.Path(__file__).parent.parent
TASK = json.loads((HERE/"task.json").read_text())
POSE = TASK["poses"][0]["resolved"]; BOLTS = POSE["bolts"]
FACES = {f["part"]: f for f in POSE["mounting_faces"]}
step = sys.argv[1]; h = float(sys.argv[2]) if len(sys.argv) > 2 else 1.5
MODE = sys.argv[3] if len(sys.argv) > 3 else "rings"
E, NU = 2500.0, 0.35
part = import_step(step)
bb = part.bounding_box()
x0, y0, z0 = bb.min.X - 0.01, bb.min.Y - 0.01, bb.min.Z - 0.01
nx, ny, nz = [int(math.ceil((a - b) / h)) + 1 for a, b in ((bb.max.X, x0), (bb.max.Y, y0), (bb.max.Z, z0))]
t0 = time.time()
inter = IntCurvesFace_ShapeIntersector(); inter.Load(part.wrapped, 1e-6)
occ = np.zeros((nx, ny, nz), bool)
zc = z0 + (np.arange(nz) + 0.5) * h
for i in range(nx):
    for j in range(ny):
        lin = gp_Lin(gp_Pnt(x0 + (i + .5) * h, y0 + (j + .5) * h, z0 - 10), gp_Dir(0, 0, 1))
        inter.Perform(lin, -1e9, 1e9)
        ws = sorted(inter.WParameter(k) + z0 - 10 for k in range(1, inter.NbPnt() + 1))
        # dedupe
        w2 = []
        for w in ws:
            if w2 and abs(w - w2[-1]) < 1e-6: continue
            w2.append(w)
        if len(w2) % 2: continue
        for a, b in zip(w2[::2], w2[1::2]):
            occ[i, j, (zc > a) & (zc < b)] = True
print("voxels", occ.sum(), "vol cm3", occ.sum() * h**3 / 1000, "t", round(time.time() - t0, 1))
# element stiffness (Wilson H8 incompatible modes), cube side h
def ke():
    lam = E * NU / ((1 + NU) * (1 - 2 * NU)); mu = E / (2 * (1 + NU))
    D = np.zeros((6, 6)); D[:3, :3] = lam; D[np.arange(3), np.arange(3)] += 2 * mu; D[3:, 3:] = np.eye(3) * mu
    sg = np.array([[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],[-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]])
    g = 1 / math.sqrt(3); K = np.zeros((33, 33))
    for xi in (-g, g):
        for et in (-g, g):
            for ze in (-g, g):
                p = np.array([xi, et, ze])
                dN = np.zeros((11, 3))
                for a in range(8):
                    s = sg[a]
                    dN[a] = [s[0]*(1+s[1]*et)*(1+s[2]*ze), s[1]*(1+s[0]*xi)*(1+s[2]*ze), s[2]*(1+s[0]*xi)*(1+s[1]*et)]
                    dN[a] /= 8
                dN[8] = [-2*xi, 0, 0]; dN[9] = [0, -2*et, 0]; dN[10] = [0, 0, -2*ze]
                dN *= 2 / h
                B = np.zeros((6, 33))
                for a in range(11):
                    dx, dy, dz = dN[a]
                    B[0, 3*a] = dx; B[1, 3*a+1] = dy; B[2, 3*a+2] = dz
                    B[3, 3*a] = dy; B[3, 3*a+1] = dx; B[4, 3*a+1] = dz; B[4, 3*a+2] = dy; B[5, 3*a] = dz; B[5, 3*a+2] = dx
                K += B.T @ D @ B * (h/2)**3
    Kuu, Kua, Kaa = K[:24, :24], K[:24, 24:], K[24:, 24:]
    return Kuu - Kua @ np.linalg.solve(Kaa, Kua.T), sg
Ke, sg = ke()
idx = np.argwhere(occ)
nid = -np.ones((nx + 1, ny + 1, nz + 1), int)
off = (sg + 1) // 2
conn = np.zeros((len(idx), 8), int)
for a in range(8):
    c = idx + off[a]; nid[c[:, 0], c[:, 1], c[:, 2]] = 0
nodes = np.argwhere(nid == 0); nid[nodes[:, 0], nodes[:, 1], nodes[:, 2]] = np.arange(len(nodes))
for a in range(8):
    c = idx + off[a]; conn[:, a] = nid[c[:, 0], c[:, 1], c[:, 2]]
X = np.array([x0, y0, z0]) + nodes * h
N = len(nodes); print("nodes", N)
dof = (conn[:, :, None] * 3 + np.arange(3)).reshape(len(idx), 24)
rows = np.repeat(dof, 24, axis=1).ravel(); cols = np.tile(dof, (1, 24)).ravel()
K = sp.csr_matrix((np.tile(Ke.ravel(), len(idx)), (rows, cols)), shape=(3 * N, 3 * N))
# boundary sets
def near_bolts(part_name, mode):
    f = FACES[part_name]; n = np.array(f["normal"]); o = np.array(f["origin"])
    dist = (X - o) @ n
    sel = np.zeros(N, bool)
    for b in BOLTS:
        if b["id"][0] != part_name: continue
        p = np.array(b["position"]); ax = np.array(b["axis"])
        v = X - p; al = v @ ax; r = np.linalg.norm(v - np.outer(al, ax), axis=1)
        sel |= (r <= b["head_diameter"] / 2 + 0.3 * h) & (al < 0.75 * h)      # seat ring
        sel |= (r <= b["clearance_diameter_range"][0] / 2 + 0.8 * h) & (al < 8) & (al > -1)  # hole wall
    if mode == "seat": sel |= dist < 0.6 * h
    return sel
fixA = near_bolts("A", MODE); slB = near_bolts("B", MODE)
print("fixed A nodes", fixA.sum(), "B nodes", slB.sum())
rc = np.array(FACES["B"]["origin"])
free = ~(fixA | slB); nf = free.sum()
fmap = -np.ones(N, int); fmap[free] = np.arange(nf)
# T: full dofs -> [free dofs, 6 master]
r_, c_, v_ = [], [], []
fn = np.where(free)[0]
for k in range(3):
    r_.append(fn * 3 + k); c_.append(fmap[fn] * 3 + k); v_.append(np.ones(nf))
sn = np.where(slB)[0]; d = X[sn] - rc
for k in range(3):
    r_.append(sn * 3 + k); c_.append(np.full(len(sn), 3 * nf + k)); v_.append(np.ones(len(sn)))
# u = U + th x d
cr = {(0, 1): (2, 1.0), (0, 2): (1, -1.0), (1, 0): (2, -1.0), (1, 2): (0, 1.0), (2, 0): (1, 1.0), (2, 1): (0, -1.0)}
for (k, m), (comp, sgn) in cr.items():   # u_k += sgn * th_m * d_comp
    r_.append(sn * 3 + k); c_.append(np.full(len(sn), 3 * nf + 3 + m)); v_.append(sgn * d[:, comp])
T = sp.csr_matrix((np.concatenate(v_), (np.concatenate(r_), np.concatenate(c_))), shape=(3 * N, 3 * nf + 6))
Kr = (T.T @ K @ T).tocsc()
t0 = time.time()
lu = spl.splu(Kr)
C = np.zeros((6, 6))
for k in range(6):
    f = np.zeros(3 * nf + 6); f[3 * nf + k] = 1; C[:, k] = lu.solve(f)[3 * nf:]
print("solve t", round(time.time() - t0, 1))
span = TASK["stiffness"]["per_mount"][0]["span_mm"]
np.set_printoptions(precision=4, suppress=True, linewidth=200)
wt, vt = np.linalg.eigh(C[:3, :3]); wr, vr = np.linalg.eigh(C[3:, 3:])
print("E", E, "translational k (N/mm) per principal dir", 1 / wt, "dirs\n", vt.T)
print("rotational kθ (N mm/rad)", 1 / wr, " -> equivalent k =", 1 / (wr * span**2), "axes\n", vr.T)
# moment -> displacement of B centre also (coupling)
print("k for moment via centre displacement:", [1 / (np.linalg.norm(C[:3, 3 + m]) * span) for m in range(3)])
