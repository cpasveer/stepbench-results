# Own voxel FE estimate of the STIFFNESS check (scratch tool, not part of the answer).
# usage: STEP=output.step H=1.5 freecadcmd -c "exec(open('scratch/vfe.py').read())"
import os, json, math, time
import numpy as np, scipy.sparse as sp, scipy.sparse.linalg as spl
import Part, FreeCAD as App
V = App.Vector
T = json.load(open("task.json"))
R = T["poses"][0]["resolved"]; F = R["mounting_faces"]
OA = np.array(F[0]["origin"]); NA = np.array(F[0]["normal"]); SA = np.array([NA[1], -NA[0], 0.0])
OB = np.array(F[1]["origin"]); NB = np.array(F[1]["normal"]); UB = np.array([-NB[1], NB[0], 0.0])
ZZ = np.array([0, 0, 1.0])
ROT = np.vstack([NA, SA, ZZ])          # world -> A frame
def toA(p): return (np.asarray(p) - OA) @ ROT.T
step = os.environ.get("STEP", "output.step"); h = float(os.environ.get("H", "1.5"))
shape = Part.read(step)
vs, tris = shape.tessellate(0.03)
P = toA(np.array([[v.x, v.y, v.z] for v in vs])); tris = np.array(tris)
lo = P.min(0) - 1e-6; lo[0] = 0.0; hi = P.max(0)
n = np.ceil((hi - lo) / h - 1e-6).astype(int)
cx = lo[0] + (np.arange(n[0]) + .5) * h; cy = lo[1] + (np.arange(n[1]) + .5) * h; cz = lo[2] + (np.arange(n[2]) + .5) * h
occ = np.zeros(n, bool)
A_, B_, C_ = P[tris[:, 0]], P[tris[:, 1]], P[tris[:, 2]]
# ray cast along z for each (x, s) column
d = (B_[:, 0] - A_[:, 0]) * (C_[:, 1] - A_[:, 1]) - (B_[:, 1] - A_[:, 1]) * (C_[:, 0] - A_[:, 0])
ok = np.abs(d) > 1e-9
A_, B_, C_, d = A_[ok], B_[ok], C_[ok], d[ok]
for i, x in enumerate(cx):
    m = (np.minimum(np.minimum(A_[:, 0], B_[:, 0]), C_[:, 0]) <= x) & (np.maximum(np.maximum(A_[:, 0], B_[:, 0]), C_[:, 0]) >= x)
    a, b, c, dd = A_[m], B_[m], C_[m], d[m]
    for j, y in enumerate(cy):
        w1 = ((b[:, 0] - x) * (c[:, 1] - y) - (b[:, 1] - y) * (c[:, 0] - x)) / dd
        w2 = ((c[:, 0] - x) * (a[:, 1] - y) - (c[:, 1] - y) * (a[:, 0] - x)) / dd
        w3 = 1 - w1 - w2
        hit = (w1 >= 0) & (w2 >= 0) & (w3 >= 0)
        if not hit.any(): continue
        zs = np.sort(w1[hit] * a[hit, 2] + w2[hit] * b[hit, 2] + w3[hit] * c[hit, 2])
        zs = zs[np.concatenate([[True], np.diff(zs) > 1e-6])]
        cnt = np.searchsorted(zs, cz)
        occ[i, j, :] = (cnt % 2) == 1
print("voxels", occ.sum(), "vol", occ.sum() * h ** 3, "true", shape.Volume)
# hex8 stiffness (unit cube size h, E=1)
nu = 0.35
def hexK(h, E=1.0):
    lam = E * nu / ((1 + nu) * (1 - 2 * nu)); mu = E / (2 * (1 + nu))
    D = np.zeros((6, 6)); D[:3, :3] = lam; D[np.arange(3), np.arange(3)] += 2 * mu; D[3:, 3:] = np.eye(3) * mu
    nodes = np.array([[i, j, k] for i in (0, 1) for j in (0, 1) for k in (0, 1)], float) * 2 - 1
    g = 1 / math.sqrt(3); K = np.zeros((24, 24))
    Bs = []; 
    for gx in (-g, g):
        for gy in (-g, g):
            for gz in (-g, g):
                dN = np.array([[nd[0] * (1 + nd[1] * gy) * (1 + nd[2] * gz), nd[1] * (1 + nd[0] * gx) * (1 + nd[2] * gz), nd[2] * (1 + nd[0] * gx) * (1 + nd[1] * gy)] for nd in nodes]) / 8 * (2 / h)
                B = np.zeros((6, 24))
                for a in range(8):
                    B[0, 3 * a] = dN[a, 0]; B[1, 3 * a + 1] = dN[a, 1]; B[2, 3 * a + 2] = dN[a, 2]
                    B[3, 3 * a] = dN[a, 1]; B[3, 3 * a + 1] = dN[a, 0]
                    B[4, 3 * a + 1] = dN[a, 2]; B[4, 3 * a + 2] = dN[a, 1]
                    B[5, 3 * a] = dN[a, 2]; B[5, 3 * a + 2] = dN[a, 0]
                Bs.append(B)
    # selective reduced integration: volumetric + shear from mean B (B-bar-like), deviatoric normal full
    Bm = sum(Bs) / 8
    for B in Bs:
        Bb = B.copy(); Bb[3:] = Bm[3:]          # shear terms reduced (cures bending lock)
        K += Bb.T @ D @ Bb * (h / 2) ** 3
    # hourglass safety: blend with a little full integration
    Kf = sum(B.T @ D @ B for B in Bs) * (h / 2) ** 3
    return 0.9 * K + 0.1 * Kf
Ke = hexK(h)
idx = np.argwhere(occ)
nid = -np.ones((n[0] + 1, n[1] + 1, n[2] + 1), int)
corners = np.array([[i, j, k] for i in (0, 1) for j in (0, 1) for k in (0, 1)])
en = idx[:, None, :] + corners[None]
flat = en[..., 0] * (n[1] + 1) * (n[2] + 1) + en[..., 1] * (n[2] + 1) + en[..., 2]
uq, inv = np.unique(flat, return_inverse=True); inv = inv.reshape(flat.shape)
nn = len(uq)
ni = uq // ((n[1] + 1) * (n[2] + 1)); nj = (uq // (n[2] + 1)) % (n[1] + 1); nk = uq % (n[2] + 1)
X = np.stack([lo[0] + ni * h, lo[1] + nj * h, lo[2] + nk * h], 1)     # node coords, A frame
dofs = (inv[:, :, None] * 3 + np.arange(3)[None, None]).reshape(len(idx), 24)
rows = np.repeat(dofs, 24, 1).ravel(); cols = np.tile(dofs, (1, 24)).ravel()
pad = np.pad(occ, 1)
inner = np.ones_like(occ)
for ax in range(3):
    for sh in (-1, 1):
        inner &= np.roll(pad, sh, ax)[1:-1, 1:-1, 1:-1]
core = inner[occ]
FC = float(os.environ.get("FCORE", "1.0")); SK = float(os.environ.get("SKIN", "1.2"))
esk = min(1.0, SK / h) * (1 - FC) + FC
emod = np.where(core, FC, esk)
print("core fraction", core.mean(), "skin modulus factor", esk)
K = sp.coo_matrix(((Ke.ravel()[None, :] * emod[:, None]).ravel(), (rows, cols)), shape=(nn * 3, nn * 3)).tocsr()
# B frame coordinates of nodes
Xw = X @ ROT + OA
wB = (Xw - OB) @ NB; uB = (Xw - OB) @ UB; zB = Xw[:, 2]
bolts = R["bolts"]
def near_bolt(names, rad, depth):
    m = np.zeros(nn, bool)
    for b in bolts:
        if b["id"][0] != names: continue
        p = np.array(b["position"]); ax = np.array(b["axis"])
        r = Xw - p; t = r @ ax; rr = np.linalg.norm(r - np.outer(t, ax), axis=1)
        m |= (rr <= rad(b)) & (t <= depth)
    return m
seatA = X[:, 0] < 0.5 * h
seatB = (wB < 0.75 * h) & (zB < 29.5)
res = {}
span = T["stiffness"]["per_mount"][0]["span_mm"]
def report(tag, Cm, extra=""):
    Cff = Cm[:3, :3]; Ctm = Cm[3:, 3:]
    wf = np.linalg.eigvalsh((Cff + Cff.T) / 2); wm = np.linalg.eigvalsh((Ctm + Ctm.T) / 2)
    kf = 1 / wf.max(); km = 1 / (wm.max() * span * span)
    S = np.diag([1, 1, 1, span, span, span]); Cs = S @ Cm @ S
    kc = 1 / np.linalg.eigvalsh((Cs + Cs.T) / 2).max()
    ev, evec = np.linalg.eigh((Cff + Cff.T) / 2); mv = np.linalg.eigh((Ctm + Ctm.T) / 2)[1][:, -1]
    print("%-14s k/E: force %.4g moment %.4g combined %.4g | Fdir(A) %s Maxis(A) %s %s" % (tag, kf, km, kc, np.round(ROT @ evec[:, -1], 2), np.round(ROT @ mv, 2), extra))
HY = os.environ.get("HYP", "H1,H2,H3,H4").split(",")
ringA = (seatA & near_bolt("A", lambda b: b["head_diameter"] / 2 + 0.5 * h, 1e9)) | near_bolt("A", lambda b: b["clearance_diameter_range"][0] / 2 + 0.8 * h, 20.0)
ringB = (seatB & near_bolt("B", lambda b: b["head_diameter"] / 2 + 0.5 * h, 1e9)) | near_bolt("B", lambda b: b["clearance_diameter_range"][0] / 2 + 0.8 * h, 20.0)
holeA = near_bolt("A", lambda b: b["clearance_diameter_range"][0] / 2 + 0.8 * h, 20.0)
holeB = near_bolt("B", lambda b: b["clearance_diameter_range"][0] / 2 + 0.8 * h, 20.0)
sets = {"H1": (seatA | holeA, seatB | holeB, "rigid"), "H2": (ringA, ringB, "rigid"),
        "H3": (seatA | holeA, seatB | holeB, "rbe3"), "H4": (ringA, ringB, "rbe3"), "H5": (seatA | holeA, ringB, "rbe3"), "H6": (seatA | holeA, ringB, "rigid")}
def rbmap(tb):
    r = Xw[tb] - OB
    G = np.zeros((len(tb) * 3, 6))
    for a in range(3):
        G[a::3, a] = 1
    G[0::3, 4] = r[:, 2]; G[0::3, 5] = -r[:, 1]
    G[1::3, 3] = -r[:, 2]; G[1::3, 5] = r[:, 0]
    G[2::3, 3] = r[:, 1]; G[2::3, 4] = -r[:, 0]
    return (G.reshape(len(tb), 3, 6).transpose(0, 2, 1) @ ROT.T).transpose(0, 2, 1).reshape(len(tb) * 3, 6)
for hyp in HY:
    fixA, tieB, kind = sets[hyp]
    tieB = tieB & ~fixA
    tb = np.flatnonzero(tieB); td = (tb[:, None] * 3 + np.arange(3)).ravel(); Gl = rbmap(tb)
    if kind == "rigid":
        free = ~(fixA | tieB); fd = np.flatnonzero(np.repeat(free, 3))
        Tm = sp.bmat([[sp.identity(len(fd)), None], [None, sp.csr_matrix(Gl)]]).tocsr()
        order = np.concatenate([fd, td])
        Kr = (Tm.T @ K[order][:, order] @ Tm).tocsc()
        rhs = np.zeros((Kr.shape[0], 6)); rhs[-6:, :] = np.eye(6)
        Cm = spl.splu(Kr).solve(rhs)[-6:, :]
        report(hyp + " " + kind, Cm, "fixA %d tieB %d" % (fixA.sum(), tieB.sum()))
    else:
        fd = np.flatnonzero(np.repeat(~fixA, 3)); pos = -np.ones(nn * 3, int); pos[fd] = np.arange(len(fd))
        Wm = np.linalg.solve(Gl.T @ Gl, Gl.T)            # 6 x 3n  (least-squares rigid fit)
        rhs = np.zeros((len(fd), 6)); rhs[pos[td], :] = Wm.T
        xx = spl.splu(K[fd][:, fd].tocsc()).solve(rhs)
        Cm = Wm @ xx[pos[td], :]
        report(hyp + " " + kind, Cm, "fixA %d tieB %d" % (fixA.sum(), tieB.sum()))
        # worst node displacement under unit force (for info)
