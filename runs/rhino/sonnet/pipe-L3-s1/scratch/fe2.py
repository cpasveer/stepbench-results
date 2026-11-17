import json, sys, time, math
import numpy as np

TASK = json.load(open('./task.json'))
PB = TASK['poses'][0]['placements']['B']; PA = TASK['poses'][0]['placements']['A']
O = np.array(PA['origin']); R = np.array(PA['rotation'])
Bo = R.T @ (np.array(PB['origin']) - O); Rb = np.array(PB['rotation'])
U = R.T @ Rb[:, 0]; V = R.T @ Rb[:, 1]; N = R.T @ Rb[:, 2]
EX = np.array([1., 0, 0])
a1 = np.array([0, -0.573576436, 0.819152044]); a2 = np.array([0, 0.573576436, 0.819152044])
E_EFF = 2075.0

def bp(u, v, n): return Bo + u * U + v * V + n * N

class Grid:
    def __init__(self, H=1.0):
        self.H = H
        self.lo = np.array([-26.0, -32.0, 20.0]); self.hi = np.array([26.0, 48.0, 125.0])
        self.n = np.ceil((self.hi - self.lo) / H).astype(int)
        xs = self.lo[0] + (np.arange(self.n[0]) + 0.5) * H
        ys = self.lo[1] + (np.arange(self.n[1]) + 0.5) * H
        zs = self.lo[2] + (np.arange(self.n[2]) + 0.5) * H
        X, Y, Z = np.meshgrid(xs, ys, zs, indexing='ij')
        self.P = np.stack([X, Y, Z], -1)
        rel = self.P - Bo
        self.u = rel @ U; self.v = rel @ V; self.nn = rel @ N
        self.r = np.hypot(Y, Z)

def build(g, p):
    P = g.P; X = P[..., 0]; Y = P[..., 1]; Z = P[..., 2]
    occ = np.zeros(X.shape, bool)
    E0 = bp(-17.5, 0, 0); Wd = np.cross(V, EX); Wd /= np.linalg.norm(Wd)
    XBA = E0[0] + math.tan(math.radians(p.get('TILT', 0.0))) * ((P - E0) @ Wd) if 'TILT' in p else p['XB']
    # pads
    for row, a, top in ((1, a1, p['PAD_XT1']), (2, a2, p['PAD_XT2'])):
        tg = np.cross(a, EX)
        ac = P @ a; tc = P @ tg
        occ |= (np.abs(tc) <= p['PAD_HW']) & (ac >= 30) & (ac <= 38) & (X >= XBA) & (X <= top)
    # strip
    al = np.degrees(np.arctan2(Y, Z))
    occ |= (np.abs(al) <= 35) & (g.r <= p['BR_RO']) & (X >= XBA) & (X <= p['TOP_W'])
    # wall
    q0 = np.array([p['Q0y'], p['Q0z']])
    E = bp(p['WALL_U_END'], p.get('WALL_V_END',0.0), 0); q1 = np.array([E[1], E[2]])
    d = q1 - q0; L = np.linalg.norm(d); d /= L; lat = np.array([-d[1], d[0]])
    yz = np.stack([Y, Z], -1) - q0
    s = yz @ d; lt = yz @ lat
    s_f = L * 0.55
    wd = np.where(s >= s_f, p['WALL_W'], p['WALL_W'] + (p['ROOT_W'] - p['WALL_W']) * (s_f - s) / (s_f + 8.0))
    wtop=p.get('W_TOP',None)
    if wtop is not None:
        frac=np.clip((p['TOP_W']-X)/(p['TOP_W']-p['XB']),0,1)
        wd=wtop+(wd-wtop)*frac
    occ |= (s >= -8) & (s <= L) & (np.abs(lt) <= wd / 2) & (X >= XBA) & (X <= p['TOP_W'] - p.get('SLOPE',0.0)*np.maximum(s,0.0))
    # block
    ctop = bp(-17.5 + p['T'] * p['DRAFT'], 0, p['T'])
    xbt = ctop[0] - p['BLOCK_BELOW']
    c1 = bp(-17.5, -17.5, 0); c2 = bp(-17.5, 17.5, 0)
    poly = [(q0 + d * (L - 3) - lat * p['WALL_W'] / 2), (q0 + d * L - lat * p['WALL_W'] / 2), np.array([c1[1], c1[2]]), np.array([c2[1], c2[2]]), (q0 + d * L + lat * p['WALL_W'] / 2), (q0 + d * (L - 3) + lat * p['WALL_W'] / 2)]
    # ensure c1 on -lat side
    if (np.array([c1[1], c1[2]]) - q0) @ lat > (np.array([c2[1], c2[2]]) - q0) @ lat:
        poly[2], poly[3] = poly[3], poly[2]
    from shapely.geometry import Polygon
    from shapely import contains_xy
    pg = Polygon(poly)
    inb = contains_xy(pg, Y, Z)
    occ |= inb & (X >= XBA) & (X <= p.get('BLOCK_TOP', xbt)) & (g.nn >= p['NMIN']) & (g.u <= p.get('BLOCK_UMAX', 99.0))
    # plate
    u, v, n = g.u, g.v, g.nn
    pl = (n >= 0) & (n <= p['T']) & (np.abs(v) <= 17.5) & (u >= -17.5 + p['DRAFT'] * n) & (u <= 17.5)
    win = (np.abs(v) < p['WIN']) & (u > -p['WIN'])
    pl &= ~win
    occ |= pl
    # pipe removal
    occ &= g.r >= 34.0
    for extra in p.get('extra', []):
        occ |= extra(g)
    return occ

def default_params():
    return dict(PAD_XT1=22.5, PAD_XT2=23.5, PAD_HW=6.5, XB=-23.5, TOP_W=-6.0, BR_RO=36.5, Q0y=-4.0, Q0z=30.0,
                WALL_U_END=-28.0, WALL_W=12.0, ROOT_W=12.0, T=7.0, DRAFT=math.tan(math.radians(10)), BLOCK_BELOW=0.0,
                NMIN=0.4, WIN=8.0)

def solve(occ, g, verbose=True, regions=True):
    H = g.H
    ex, ey, ez = np.nonzero(occ)
    nn = (g.n[0] + 1, g.n[1] + 1, g.n[2] + 1)
    def nid(i, j, k): return (i * nn[1] + j) * nn[2] + k
    corner = [(0,0,0),(1,0,0),(1,1,0),(0,1,0),(0,0,1),(1,0,1),(1,1,1),(0,1,1)]
    en = np.stack([nid(ex + c[0], ey + c[1], ez + c[2]) for c in corner], 1)
    used, inv = np.unique(en.ravel(), return_inverse=True)
    en = inv.reshape(en.shape)
    nnodes = len(used)
    ui = used // (nn[1] * nn[2]); uj = (used // nn[2]) % nn[1]; uk = used % nn[2]
    X = g.lo + np.stack([ui, uj, uk], 1) * H
    nu = 0.35
    Dm = (1.0 / ((1 + nu) * (1 - 2 * nu))) * np.array([[1-nu, nu, nu, 0,0,0],[nu,1-nu,nu,0,0,0],[nu,nu,1-nu,0,0,0],[0,0,0,(1-2*nu)/2,0,0],[0,0,0,0,(1-2*nu)/2,0],[0,0,0,0,0,(1-2*nu)/2]])
    sgn = np.array([[-1,-1,-1],[1,-1,-1],[1,1,-1],[-1,1,-1],[-1,-1,1],[1,-1,1],[1,1,1],[-1,1,1]], float)
    gg = 1 / np.sqrt(3)
    Ke = np.zeros((24, 24))
    for gx in (-gg, gg):
        for gy in (-gg, gg):
            for gz in (-gg, gg):
                dN = np.zeros((8, 3))
                for a in range(8):
                    dN[a] = [sgn[a,0]*(1+sgn[a,1]*gy)*(1+sgn[a,2]*gz)/8, sgn[a,1]*(1+sgn[a,0]*gx)*(1+sgn[a,2]*gz)/8, sgn[a,2]*(1+sgn[a,0]*gx)*(1+sgn[a,1]*gy)/8]
                dN = dN * (2.0 / H)
                B = np.zeros((6, 24))
                for a in range(8):
                    B[0, 3*a] = dN[a,0]; B[1, 3*a+1] = dN[a,1]; B[2, 3*a+2] = dN[a,2]
                    B[3, 3*a] = dN[a,1]; B[3, 3*a+1] = dN[a,0]
                    B[4, 3*a+1] = dN[a,2]; B[4, 3*a+2] = dN[a,1]
                    B[5, 3*a] = dN[a,2]; B[5, 3*a+2] = dN[a,0]
                Ke += B.T @ Dm @ B * (H / 2) ** 3
    edofs = (en[:, :, None] * 3 + np.arange(3)[None, None, :]).reshape(len(en), 24)
    ndof = 3 * nnodes
    def Kmul(u):
        fe = u[edofs] @ Ke
        return np.bincount(edofs.ravel(), weights=fe.ravel(), minlength=ndof)
    diagK = np.bincount(edofs.ravel(), weights=np.tile(np.diag(Ke), len(en)), minlength=ndof)
    r = np.hypot(X[:, 1], X[:, 2])
    fixed_nodes = np.nonzero(r < 34.0 + 0.9 * H)[0]
    rel = X - Bo
    nB = rel @ N; uB = rel @ U; vB = rel @ V
    wash_nodes = np.nonzero((nB < 0.5 * H + 0.2) & (np.abs(uB) < 18) & (np.abs(vB) < 18))[0]
    wash_nodes = np.setdiff1d(wash_nodes, fixed_nodes)
    isfixed = np.zeros(ndof, bool)
    for k in range(3): isfixed[3 * fixed_nodes + k] = True
    Lscale = 20.0
    Rmat = np.zeros((len(wash_nodes), 3, 6))
    for i, nd in enumerate(wash_nodes):
        p = X[nd] - Bo
        Rmat[i, :, 0:3] = np.eye(3)
        Rmat[i, :, 3:6] = np.array([[0, p[2], -p[1]], [-p[2], 0, p[0]], [p[1], -p[0], 0]]) / Lscale
    widx = (3 * wash_nodes[:, None] + np.arange(3)[None, :])
    isw = np.zeros(ndof, bool); isw[widx.ravel()] = True
    free = ~isfixed & ~isw
    fidx = np.nonzero(free)[0]; nf = len(fidx)
    def scatter(z):
        u = np.zeros(ndof); u[fidx] = z[:nf]; u[widx.ravel()] = (Rmat @ z[nf:]).ravel(); return u
    def op(z):
        f = Kmul(scatter(z))
        out = np.empty_like(z); out[:nf] = f[fidx]
        out[nf:] = np.einsum('nij,ni->j', Rmat, f[widx.ravel()].reshape(-1, 3))
        return out
    pre = np.ones(nf + 6); pre[:nf] = 1.0 / diagK[fidx]
    dq = np.zeros(6)
    for i in range(len(wash_nodes)):
        dq += np.einsum('ij,i->j', Rmat[i] ** 2, diagK[widx[i]])
    pre[nf:] = 1.0 / dq
    def cg(b, tol=1e-6, maxit=8000):
        x = np.zeros_like(b); rr = b.copy(); zz = pre * rr; pv = zz.copy(); rz = rr @ zz; b0 = np.linalg.norm(b)
        for it in range(maxit):
            Ap = op(pv); a = rz / (pv @ Ap); x += a * pv; rr -= a * Ap
            if np.linalg.norm(rr) < tol * b0: break
            zz = pre * rr; rz2 = rr @ zz; pv = zz + (rz2 / rz) * pv; rz = rz2
        return x, it
    C = np.zeros((6, 6)); sols = []
    for i in range(6):
        b = np.zeros(nf + 6); b[nf + i] = 1.0
        x, it = cg(b); C[:, i] = x[nf:]; sols.append(x)
    span = 78.15
    S = np.diag([1, 1, 1, span / Lscale, span / Lscale, span / Lscale])
    Cg = S @ C @ S
    lam = np.linalg.eigvalsh((Cg + Cg.T) / 2)
    k1 = 1 / lam.max()
    res = dict(k=k1 * E_EFF, k1=k1, vol=occ.sum() * H ** 3, nwash=len(wash_nodes), nfixed=len(fixed_nodes))
    if regions:
        cen = X[en].mean(1); relc = cen - Bo
        nC = relc @ N; uC = relc @ U; vC = relc @ V; rC = np.hypot(cen[:, 1], cen[:, 2])
        reg = np.full(len(en), 'wall', dtype=object)
        reg[(rC < 38.6)] = 'pad'
        reg[(uC > -17.5) & (nC > -0.5) & (rC > 38.6)] = 'plate'
        reg[(uC <= -17.5) & (uC > -31) & (np.abs(vC) < 18.5) & (nC < 12) & (rC > 38.6)] = 'block'
        # worst mode eigenvector
        w, vecs = np.linalg.eigh((Cg + Cg.T) / 2)
        wvec = vecs[:, -1]
        # load = S^-1 ... dual load vector for worst mode: generalized load f_g = wvec -> q-load = S wvec
        b = np.zeros(nf + 6); b[nf:] = S @ wvec
        x, it = cg(b)
        u = scatter(x); ue = u[edofs]
        e = 0.5 * np.einsum('ni,ij,nj->n', ue, Ke, ue)
        res['share'] = {kk: round(float(e[reg == kk].sum() / e.sum()), 2) for kk in ['pad', 'wall', 'block', 'plate']}
        res['mode'] = np.round(wvec, 2)
    # printed estimate
    def erode(o, sy, sx):
        r_ = o.copy()
        for _ in range(sy):
            r2 = r_.copy()
            r2[:, 1:, :] &= r_[:, :-1, :]; r2[:, :-1, :] &= r_[:, 1:, :]; r2[:, :, 1:] &= r_[:, :, :-1]; r2[:, :, :-1] &= r_[:, :, 1:]
            r_ = r2
        for _ in range(sx):
            r2 = r_.copy(); r2[1:] &= r_[:-1]; r2[:-1] &= r_[1:]; r_ = r2
        return r_
    core = erode(occ, max(1, int(round(1.2 / H))), max(1, int(round(0.8 / H))))
    vol = occ.sum() * H ** 3; vc = core.sum() * H ** 3
    res['printed'] = vol - vc + 0.2 * vc
    return res

if __name__ == '__main__':
    g = Grid(1.0)
    p = default_params()
    t0 = time.time()
    occ = build(g, p)
    r = solve(occ, g)
    print({k: (round(v, 4) if isinstance(v, float) else v) for k, v in r.items()}, round(time.time() - t0, 1))
