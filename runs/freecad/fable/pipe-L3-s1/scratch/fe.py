"""Rough voxel FE of output.step (own estimate of the STIFFNESS figure).
usage: FE_ARGS="step hx hy hz beam_w beam_h" freecadcmd scratch/fe.py"""
import json, math, os, sys, time
import numpy as np, scipy.sparse as sp, scipy.sparse.linalg as spl
import FreeCAD as App, Part
V = App.Vector
args = os.environ.get("FE_ARGS", "output.step 2 2 2 7.5 23.5").split()
step = args[0]; hx, hy, hz = map(float, args[1:4]); BW = float(args[4]); BH = float(args[5])
T = json.load(open("task.json")); pose = T["poses"][0]
ra = pose["placements"]["A"]["rotation"]; D = V(ra[0][0], ra[1][0], ra[2][0]); D.normalize()
O = V(*pose["placements"]["B"]["origin"]); N = V(*pose["resolved"]["mounting_faces"][0]["normal"])
Q = N - D * N.dot(D); Q.normalize(); U = Q.cross(D)
cp = V(*pose["placements"]["A"]["origin"]) - O
cx, cy = cp.dot(U), cp.dot(Q)
cn, sn = N.dot(Q), N.dot(D)
span = T["stiffness"]["per_mount"][0]["span_mm"]
s = Part.read(step)
pts = [(v.Point - O) for v in s.Vertexes]
lo = [min(p.dot(a) for p in pts) for a in (U, Q, D)]; hi = [max(p.dot(a) for p in pts) for a in (U, Q, D)]
# include cylinder extents roughly
zb = hi[2]
# grid aligned: u lines through +-BW, z lines through zb and zb-BH
hx = BW / max(1, round(BW / hx)); hz = BH / max(1, round(BH / hz))
x0 = -BW - hx * math.ceil((-BW - lo[0]) / hx + 1); nx = int(math.ceil((hi[0] - x0) / hx)) + 1
z0 = zb - hz * math.ceil((zb - lo[2]) / hz); nz = int(round((zb - z0) / hz))
y0 = lo[1] - 0.01; ny = int(math.ceil((hi[1] - y0) / hy))
t0 = time.time()
solid = np.zeros((nx, ny, nz), bool)
import hashlib
key = hashlib.md5((open(step,"rb").read() + repr((hx,hy,hz,x0,y0,z0,nx,ny,nz)).encode())).hexdigest()
cache = "scratch/vox_%s.npy" % key
if os.path.exists(cache):
    solid = np.load(cache)
for i in range(nx if not os.path.exists(cache) else 0):
    for j in range(ny):
        x = x0 + (i + .5) * hx; y = y0 + (j + .5) * hy
        for k in range(nz):
            z = z0 + (k + .5) * hz
            if s.isInside(O + U * x + Q * y + D * z, 1e-6, True):
                solid[i, j, k] = True
np.save(cache, solid)
print("voxels", solid.sum(), "vol", solid.sum() * hx * hy * hz, "t", time.time() - t0)
# hex8 stiffness
E, nu = 1.0, 0.35
EZ = float(os.environ.get("EZ", 1)); GZ = float(os.environ.get("GZ", 1)); TIE = float(os.environ.get("TIE", -1))
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
nid = -np.ones((nx + 1, ny + 1, nz + 1), int)
el = np.argwhere(solid)
conn = np.zeros((len(el), 8), int)
cnt = 0
for n, (di, dj, dk) in enumerate(nodes):
    pass
used = np.zeros((nx + 1, ny + 1, nz + 1), bool)
for (di, dj, dk) in nodes:
    used[el[:, 0] + di, el[:, 1] + dj, el[:, 2] + dk] = True
nid[used] = np.arange(used.sum()); nn = used.sum()
for n, (di, dj, dk) in enumerate(nodes):
    conn[:, n] = nid[el[:, 0] + di, el[:, 1] + dj, el[:, 2] + dk]
dof = (conn[:, :, None] * 3 + np.arange(3)[None, None, :]).reshape(len(el), 24)
I = np.repeat(dof, 24, axis=1).ravel(); J = np.tile(dof, (1, 24)).ravel()
K = sp.coo_matrix((np.tile(Ke.ravel(), len(el)), (I, J)), shape=(3 * nn, 3 * nn)).tocsr()
ijk = np.argwhere(used)
X = np.stack([x0 + ijk[:, 0] * hx, y0 + ijk[:, 1] * hy, z0 + ijk[:, 2] * hz], 1)
rad = np.hypot(X[:, 0] - cx, X[:, 1] - cy)
fixed = rad <= 34.0 + 0.6 * max(hx, hy)
ncoord = X[:, 1] * cn + X[:, 2] * sn
zl = -X[:, 1] * sn + X[:, 2] * cn
tied = (ncoord <= (TIE if TIE > 0 else 0.6 * max(hy, hz) + 0.5)) & (np.abs(zl) < 18) & (np.abs(X[:, 0]) < 18) & ~fixed
Xw = np.array([O.x, O.y, O.z]) + np.outer(X[:, 0], [U.x, U.y, U.z]) + np.outer(X[:, 1], [Q.x, Q.y, Q.z]) + np.outer(X[:, 2], [D.x, D.y, D.z])
def bolt_dist(prefix):
    dmin = np.full(len(X), 1e9)
    for b in pose["resolved"]["bolts"]:
        if b["id"][0] != prefix: continue
        p = np.array(b["position"]); ax = np.array(b["axis"]); r = Xw - p
        perp = r - np.outer(r @ ax, ax)
        dmin = np.minimum(dmin, np.linalg.norm(perp, axis=1))
    return dmin
dB = bolt_dist("B")
hmax = max(hx, hy, hz)
inplate = (np.abs(zl) < 18) & (np.abs(X[:, 0]) < 18) & ~fixed & (ncoord < 8)
np.set_printoptions(precision=4, suppress=True)
res_out = {}
for cfg in os.environ.get("CFGS", "full 3.5 4.5 6").split():
    if cfg == "full":
        tied = inplate & (ncoord <= 0.6 * hmax)
    else:
        rw = float(cfg)
        tied = inplate & (((ncoord <= 0.6 * hmax) & (dB <= rw)) | (dB <= 2.25 + 0.6 * hmax))
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
    sol = spl.splu(Kr).solve(rhs)
    Cm = sol[3 * nf:, :]
    Ct = Cm[:3, :3]; Cr = Cm[3:, 3:] * span * span
    wt = np.linalg.eigvalsh(Ct); wr = np.linalg.eigvalsh(Cr)
    worst = min((1 / wt).min(), (1 / wr).min())
    print("CFG %s tied %d worst %.5f | transl %s rot %s (t %.0fs)" % (cfg, tied.sum(), worst, 1 / np.diag(Ct), 1 / np.diag(Cr), time.time() - t0))
