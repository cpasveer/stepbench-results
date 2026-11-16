import sys, math, json, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).parent.parent))
from build123d import *
import submission as S

part = import_step(str(pathlib.Path(__file__).parent.parent / "output.step"))
solid = part.solids()[0] if hasattr(part, "solids") else part
fa = import_step(str(pathlib.Path(__file__).parent.parent / "fixed_A.step"))
fb = import_step(str(pathlib.Path(__file__).parent.parent / "fixed_B.step"))
print("valid", solid.is_valid, "vol", round(solid.volume,1))
vol=lambda x: (x.volume if x is not None else 0.0)
print("overlap A", round(vol(solid & fa),4), "overlap B", round(vol(solid & fb),4))

def legs_blocked(bid, r_leg=2.5, step=6):
    b = S.BOLTS[bid]
    p = tuple(b["position"]); ax = tuple(b["axis"])
    g = S.PAD_A_T if bid[0]=="A" else S.PAD_B_T
    seat = p  # on face
    bend = tuple(seat[i] + ax[i]*(g+5+22.5) for i in range(3))
    # build in-plane basis
    ref = (0,0,1.0)
    u = S.sub(ref, S.mul(ax, S.dot(ref, ax)))
    if math.sqrt(S.dot(u,u)) < 1e-6: u=(1,0,0)
    L = math.sqrt(S.dot(u,u)); u = S.mul(u, 1/L)
    v = (ax[1]*u[2]-ax[2]*u[1], ax[2]*u[0]-ax[0]*u[2], ax[0]*u[1]-ax[1]*u[0])
    blocked = {}
    for ang in range(0, 360, step):
        a = math.radians(ang)
        dvec = S.vadd(S.mul(u, math.cos(a)), S.mul(v, math.sin(a)))
        leg = Solid.make_cylinder(r_leg, 70, Plane(origin=bend, z_dir=dvec))
        inter = solid & leg
        blocked[ang] = vol(inter) > 1e-3
    # short leg
    sh = Solid.make_cylinder(2.5, 25, Plane(origin=tuple(seat[i]+ax[i]*2.5 for i in range(3)), z_dir=ax))
    short_hit = vol(solid & sh) > 1e-3
    return blocked, short_hit

for bid in S.BOLTS:
    blk, sh = legs_blocked(bid)
    free = [a for a in blk if not blk[a]]
    # longest run of consecutive free angles (wrap)
    best=0; run=0
    angs = sorted(blk)
    seq = angs + angs
    for a in seq:
        if not blk[a % 360]:
            run+=6; best=max(best,run)
        else: run=0
    print(bid, "short_leg_hit", sh, "best free sweep deg", min(best,360), "blocked count", sum(blk.values()))
