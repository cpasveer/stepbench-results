"""Voxel FE on a grid aligned with B's plate (u, n, zl); hex8 with incompatible modes.
FE2_ARGS="step tp beam_w res"  CFGS="full seat3.5 head3.5 ..." """
import json, math, os, sys, time, hashlib
import numpy as np, scipy.sparse as sp, scipy.sparse.linalg as spl
import FreeCAD as App, Part
V = App.Vector
a_ = os.environ["FE2_ARGS"].split()
step, tp, BW, res = a_[0], float(a_[1]), float(a_[2]), float(a_[3])
T = json.load(open("task.json")); pose = T["poses"][0]
ra = pose["placements"]["A"]["rotation"]; D = V(ra[0][0], ra[1][0], ra[2][0]); D.normalize()
O = V(*pose["placements"]["B"]["origin"]); N = V(*pose["resolved"]["mounting_faces"][0]["normal"]); N.normalize()
Q = N - D * N.dot(D); Q.normalize(); U = Q.cross(D); ZL = U.cross(N)
AX = (U, N, ZL)
cpipe = V(*pose["placements"]["A"]["origin"])
span = T["stiffness"]["per_mount"][0]["span_mm"]
s = Part.read(step)
pts = [(v.Point - O) for v in s.Vertexes]
lo = [min(p.dot(a) for p in pts) for a in AX]; hi = [max(p.dot(a) for p in pts) for a in AX]
hx = BW / max(1, round(BW / res)); hy = tp / max(2, round(tp / res)); hz = res
x0 = -BW - hx * math.ceil((-BW - lo[0]) / hx + 1); nx = int(math.ceil((hi[0] - x0) / hx)) + 1
y0 = 0.0; ny = int(math.ceil((hi[1] + 1) / hy))
z0 = lo[2] - 0.01; nz = int(math.ceil((hi[2] - z0) / hz)) + 1
key = hashlib.md5((open(step, "rb").read() + repr((hx, hy, hz, x0, y0, z0, nx, ny, nz)).encode())).hexdigest()
cache = "scratch/vox2_%s.npy" % key
t0 = time.time()
if os.path.exists(cache):
    solid = np.load(cache)
else:
    solid = np.zeros((nx, ny, nz), bool)
    bb = s.BoundBox
    for i in range(nx):
        for j in range(ny):
            for k in range(nz):
                p = O + U * (x0 + (i + .5) * hx) + N * (y0 + (j + .5) * hy) + ZL * (z0 + (k + .5) * hz)
                if bb.isInside(p) and s.isInside(p, 1e-6, True):
                    solid[i, j, k] = True
    np.save(cache, solid)
print("grid", nx, ny, nz, "h", hx, hy, hz, "voxels", solid.sum(), "vol", solid.sum() * hx * hy * hz, "t %.0f" % (time.time() - t0))
E, nu = 1.0, 0.35
def hexK(a, b, c):
    lam = E * nu / ((1 + nu) * (1 - 2 * nu)); mu = E / (2 * (1 + nu))
    C = np.zeros((6, 6)); C[:3, :3] = lam; C[range(3), range(3)] += 2 * mu; C[range(3, 6), range(3, 6)] = mu
    g = 1 / math.sqrt(3)
    nodes = [(i, j, k) for k in (0, 1) for j in (0, 1) for i in (0, 1)]
    Kuu = np.zeros((24, 24)); Kua = np.zeros((24, 9)); Kaa = np.zeros((9, 9))
    def Bmat(grads):
        B = np.zeros((6, 3 * len(grads)))
        for n, (dx, dy, dz) in enumerate(grads):
            B[0, 3 * n] = dx; B[1, 3 * n + 1] = dy; B[2, 3 * n + 2] = dz
            B[3, 3 * n] = dy; B[3, 3 * n + 1] = dx
            B[4, 3 * n + 1] = dz; B[4, 3 * n + 2] = dy
            B[5, 3 * n] = dz; B[5, 3 * n + 2] = dx
        return B
    for gx in (-g, g):
        for gy in (-g, g):
            for gz in (-g, g):
                gr = []
                for (i, j, k) in nodes:
                    sx, sy, sz = 2 * i - 1, 2 * j - 1, 2 * k - 1
                    gr.append((sx * (1 + sy * gy) * (1 + sz * gz) / 8 * 2 / a,
                               sy * (1 + sx * gx) * (1 + sz * gz) / 8 * 2 / b,
                               sz * (1 + sx * gx) * (1 + sy * gy) / 8 * 2 / c))
                Bu = Bmat(gr)
                Ba = Bmat([(-2 * gx * 2 / a, 0, 0), (0, -2 * gy * 2 / b, 0), (0, 0, -2 * gz * 2 / c)])
                w = a * b * c / 8
                Kuu += Bu.T @ C @ Bu * w; Kua += Bu.T @ C @ Ba * w; Kaa += Ba.T @ C @ Ba * w
    return Kuu - Kua @ np.linalg.solve(Kaa, Kua.T), nodes
Ke, nodes = hexK(hx, hy, hz)
el = np.argwhere(solid)
used = np.zeros((nx + 1, ny + 1, nz + 1), bool)
for (di, dj, dk) in nodes:
    used[el[:, 0] + di, el[:, 1] + dj, el[:, 2] + dk] = True
nid = -np.ones(used.shape, int); nn = int(used.sum()); nid[used] = np.arange(nn)
conn = np.stack([nid[el[:, 0] + di, el[:, 1] + dj, el[:, 2] + dk] for (di, dj, dk) in nodes], 1)
dof = (conn[:, :, None] * 3 + np.arange(3)[None, None, :]).reshape(len(el), 24)
I = np.repeat(dof, 24, axis=1).ravel(); J = np.tile(dof, (1, 24)).ravel()
K = sp.coo_matrix((np.tile(Ke.ravel(), len(el)), (I, J)), shape=(3 * nn, 3 * nn)).tocsr()
ijk = np.argwhere(used)
X = np.stack([x0 + ijk[:, 0] * hx, y0 + ijk[:, 1] * hy, z0 + ijk[:, 2] * hz], 1)   # (u, n, zl)
A3 = np.array([[a.x, a.y, a.z] for a in AX])
Xw = np.array([O.x, O.y, O.z]) + X @ A3
dv = np.array([D.x, D.y, D.z]); r = Xw - np.array([cpipe.x, cpipe.y, cpipe.z])
rad = np.linalg.norm(r - np.outer(r @ dv, dv), axis=1)
hmax = max(hx, hy, hz)
fixed = rad <= 34.0 + 0.6 * hmax
dB = np.full(nn, 1e9)
for b in pose["resolved"]["bolts"]:
    if b["id"][0] == "B":
        p = np.array(b["position"]); ax = np.array(b["axis"]); rr = Xw - p
        dB = np.minimum(dB, np.linalg.norm(rr - np.outer(rr @ ax, ax), axis=1))
inplate = (X[:, 1] <= tp + 1e-6) & (np.abs(X[:, 0]) < 19) & (np.abs(X[:, 2]) < 19) & ~fixed
seat = inplate & (X[:, 1] < 1e-6); headf = inplate & (np.abs(X[:, 1] - tp) < 1e-6)
hole = inplate & (dB <= 2.25 + 0.55 * max(hx, hz))
np.set_printoptions(precision=4, suppress=True)
for cfg in os.environ.get("CFGS", "full seat3.5 seat5 head3.5 hole").split():
    if cfg == "full": tied = seat.copy()
    elif cfg.startswith("seat"): tied = (seat & (dB <= float(cfg[4:]))) | hole
    elif cfg.startswith("head"): tied = (headf & (dB <= float(cfg[4:]))) | hole
    elif cfg.startswith("both"): tied = ((headf | seat) & (dB <= float(cfg[4:]))) | hole
    else: tied = hole.copy()
    free = ~fixed & ~tied
    fidx = np.where(free)[0]; nf = len(fidx)
    fd = (fidx[:, None] * 3 + np.arange(3)).ravel()
    rows = list(fd); cols = list(range(3 * nf)); vals = [1.0] * (3 * nf)
    for n in np.where(tied)[0]:
        rx, ry, rz = X[n]
        for a in range(3):
            rows.append(3 * n + a); cols.append(3 * nf + a); vals.append(1.0)
        for (a, b, v) in ((0, 1, rz), (0, 2, -ry), (1, 0, -rz), (1, 2, rx), (2, 0, ry), (2, 1, -rx)):
            rows.append(3 * n + a); cols.append(3 * nf + 3 + b); vals.append(v)
    Tm = sp.coo_matrix((vals, (rows, cols)), shape=(3 * nn, 3 * nf + 6)).tocsr()
    Kr = (Tm.T @ K @ Tm).tocsc()
    t0 = time.time()
    rhs = np.zeros((3 * nf + 6, 6)); rhs[3 * nf:, :] = np.eye(6)
    Cm = spl.splu(Kr).solve(rhs)[3 * nf:, :]
    Ct = Cm[:3, :3]; Cr = Cm[3:, 3:] * span * span
    wt = np.linalg.eigvalsh(Ct); wr = np.linalg.eigvalsh(Cr)
    Dg = np.diag([1, 1, 1, span, span, span]); w6 = np.linalg.eigvalsh(Dg @ (Cm + Cm.T) / 2 @ Dg)
    print("CFG %-8s tied %4d worst %.5f six %.5f | transl(u,n,zl) %s rot %s (%.0fs)" % (cfg, tied.sum(), min((1 / wt).min(), (1 / wr).min()), 1 / w6.max(), 1 / np.diag(Ct), 1 / np.diag(Cr), time.time() - t0))
    sys.stdout.flush()
