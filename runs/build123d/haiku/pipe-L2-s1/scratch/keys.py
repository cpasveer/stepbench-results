import sys, pathlib, math
sys.path.insert(0, str(pathlib.Path(__file__).parent.parent))
from build123d import Vector, Solid, Plane
import submission as S
part = S.build()
BOL = S.BOLTS
# each bolt: head band above the part top (grip g), key long leg band
def perp(z):
    ref = Vector(1,0,0) if abs(z.X)<0.9 else Vector(0,1,0)
    return (ref - z*ref.dot(z)).normalized()
res = {}
for bid, b in sorted(BOL.items()):
    ax = Vector(*b["axis"]).normalized(); pos = Vector(*b["position"])
    M5 = b["size"]=="M5"
    R = 70.0 if M5 else 63.0
    grip = 8.0
    head_top = grip + (5.0 if M5 else 4.0)
    z0 = pos + ax * (head_top + 0.5)
    # slab: plane at z0 extending 3.5 mm along axis
    slab = Solid.make_cylinder(R, 3.5, Plane(z0, perp(ax), ax))
    sec = part & slab
    if sec is None:
        res[bid] = "free (no material in key band)"; continue
    # sample occupancy by angle using point classification on a grid
    xdir = perp(ax); ydir = ax.cross(xdir)
    free = []
    occ = []
    for k in range(0, 360, 2):
        th = math.radians(k)
        dirv = xdir*math.cos(th) + ydir*math.sin(th)
        hit = False
        for rr in [i*1.0 for i in range(5, int(R)+1)]:
            p = z0 + ax*1.75 + dirv*rr
            if sec.is_inside(p) if hasattr(sec,'is_inside') else False:
                hit=True;break
        occ.append(hit)
    # largest free run in degrees
    run=0; best=0
    for o in occ+occ:
        run = 0 if o else run+2
        best=max(best,run)
    res[bid]=f"largest free angle {min(best,360)} deg"
for k,v_ in res.items(): print(k, v_)
