"""Flange-to-block connector (flange-L3-s1), FreeCAD script.

Concept
-------
Two upright plates (a round plate on flange A, a rectangular plate on block B)
joined by one solid core.  The core is the intersection of two prisms:
  * an "A prism" running along A's bolt axes whose cross-section keeps out of
    every A bolt's head, tool and key space (used only as far as those reach),
  * a "B prism" running along B's bolt axes doing the same for B.
So no bolt axis anywhere passes through the core, while the core stays as
large as the bolt patterns allow (high stiffness from a short, thick member).

Printing: the part is printed upside down (world +z towards the bed).  The
plates' top chords and the core's top web all lie in one plane z = Z_BED,
every other downward-facing (in print) face is a vertical wall or a >= 50 deg
roof, so no support is needed.  The small B-plate extension that reaches the
bed is set back 0.6 mm from B's face so it does not sit outside B's outline.
"""
import json
import math
import os
import sys
import traceback

import FreeCAD as App
import Part

try:
    HERE = os.path.dirname(os.path.abspath(__file__))
except NameError:
    HERE = os.getcwd()
TASK = json.load(open(os.path.join(HERE, "task.json")))
POSE = TASK["poses"][0]["resolved"]
FACES = {f["part"]: f for f in POSE["mounting_faces"]}
BOLTS = POSE["bolts"]
V = App.Vector
Z = V(0, 0, 1)

# ---------------------------------------------------------------- parameters
P = dict(HA=14.5, ZBA=9.0, WA=7.5, HB=13.0, KB=4.0, ZKB=99.0, ZSTEP=9.0,
         WB=8.0, HEXCUT=1, HBS=21.0, NOTCH=1, NIN=10.0, ANOTCH=1, ANZ=-2.0)
P.update(json.loads(os.environ.get("CONN_PARAMS", "{}")))
T_A = 4.0            # A plate thickness (grip 3.75..25)
T_B = 5.0            # B plate thickness (grip 4.5..25)
R_A = 37.25 - 1.0    # A seat radius (1 mm inside the face outline)
HB_S, HB_Z = P['HBS'], 47.0   # B seat half sizes (1 mm inside 46 x 96)
Z_BED = 36.5         # print bed plane (world z, part printed upside down)
ROOF = math.tan(math.radians(52))   # roof steepness (dz per mm sideways)
D_HOLE_A = 5.5
D_HOLE_B = 6.6
A_REACH = 44.0       # how far in front of A the A keep-outs matter
B_REACH = 45.0       # same for B

cA = V(*FACES["A"]["origin"]); nA = V(*FACES["A"]["normal"]).normalize()
cB = V(*FACES["B"]["origin"]); nB = V(*FACES["B"]["normal"]).normalize()
uA = Z.cross(nA).normalize()          # A lateral (horizontal)
sB = Z.cross(nB).normalize()          # B lateral (horizontal)


def PA(u, z, w):
    p = cA + uA * u + nA * w
    return V(p.x, p.y, z)


def PB(s, z, d):
    p = cB + sB * s + nB * d
    return V(p.x, p.y, z)


def prism(fn, pts, a, b):
    """Polygon pts (lateral, z) in a frame, extruded along that frame's
    normal from distance a to b."""
    poly = Part.makePolygon([fn(p[0], p[1], a) for p in pts] + [fn(pts[0][0], pts[0][1], a)])
    face = Part.Face(poly)
    n = (fn(0, 0, 1) - fn(0, 0, 0))
    return face.extrude(n * (b - a))


def roof_pts(web, z_web, half, z_bot, z_top=Z_BED):
    """Cross-section: web |x|<=web from z_top down to z_web, roof at ROOF
    down to |x|=half, vertical sides down to z_bot."""
    z_side = z_web - (half - web) * ROOF
    return [(-web, z_top), (-web, z_web), (-half, z_side), (-half, z_bot),
            (half, z_bot), (half, z_side), (web, z_web), (web, z_top)]


def plate_A():
    disc = Part.makeCylinder(R_A, T_A, cA, nA)
    zc = cA.z
    chord = Z_BED - zc                       # chord height above centre
    xc = math.sqrt(R_A ** 2 - chord ** 2)
    # shoulder: from chord end going down/outward at ROOF until well outside
    big = 60.0
    pts = [(-xc, Z_BED), (xc, Z_BED), (xc + big, Z_BED - big * ROOF),
           (xc + big, zc - 80), (-xc - big, zc - 80), (-xc - big, Z_BED - big * ROOF)]
    keep = prism(PA, pts, -1, T_A + 1)
    plate = disc.common(keep)
    if P['HEXCUT']:
        # straight edges tangent to the bolt bosses between neighbouring
        # bolts (all but the top pair, which carries the bed chord)
        d = 31.0   # edges meet inside the 36.25 circle: clean corners
        for k in range(6):
            a = math.radians(30 + 60 * k)          # mid-angle between bolts
            if abs(math.degrees(a) - 90) < 1:
                continue
            nrm = (math.cos(a), math.sin(a))
            t = (-nrm[1], nrm[0])
            c0 = (nrm[0] * d, nrm[1] * d)
            pts = [(c0[0] + t[0] * 60, cA.z + c0[1] + t[1] * 60),
                   (c0[0] - t[0] * 60, cA.z + c0[1] - t[1] * 60),
                   (c0[0] - t[0] * 60 + nrm[0] * 30, cA.z + c0[1] - t[1] * 60 + nrm[1] * 30),
                   (c0[0] + t[0] * 60 + nrm[0] * 30, cA.z + c0[1] + t[1] * 60 + nrm[1] * 30)]
            plate = plate.cut(prism(PA, pts, -1, T_A + 1))
    if P['ANOTCH']:
        # open the bottom between A3 and A4: legs of boss width run from the
        # A2/A5 bosses down to A3/A4 (60 deg, prints without support)
        lim = 31.0 - 10.5
        zt = P['ANZ']
        # inner leg line: 0.866|u| - 0.5 z' = lim; the two lines meet below
        # the plate at z' = -2 lim, so the cut is a triangle
        ut = (lim + 0.5 * zt) / 0.8660254
        pts = [(-ut, cA.z + zt), (0.0, cA.z - 2.0 * lim), (ut, cA.z + zt)]
        plate = plate.cut(prism(PA, pts, -1, T_A + 1))
    return plate


def plate_B():
    w = 10.0
    z_sh = Z_BED - (HB_S - w) * ROOF
    pts = [(-HB_S, cB.z - HB_Z), (HB_S, cB.z - HB_Z), (HB_S, z_sh), (w, Z_BED),
           (-w, Z_BED), (-HB_S, z_sh)]
    pl = prism(PB, pts, 0, T_B)
    # set the part above the face's 1 mm line back from the face
    z0 = cB.z + HB_Z - 0.2     # 28.8
    rec = 0.6
    # polygon in (d, z) plane, extruded along s
    def PD(d, z, s):
        return PB(s, z, d)
    cutpts = [(-1, z0 - 2.0), (rec, z0 + 2.0 * rec), (rec, Z_BED + 5), (-1, Z_BED + 5)]
    cut = prism(PD, cutpts, -40, 40)
    pl = pl.cut(cut)
    if P['NOTCH']:
        # side notches between the top and bottom bolt pairs: the upper edge
        # (facing down in use, up in print) is flat, the lower edge rises
        # outward steeply so it prints without support
        kb, hb, zs = P['KB'], P['HB'], P['ZSTEP']
        z_top = 22.0 - 6.0 - 1.0
        # lowest start of the rising lower edge that still clears B1/B2's
        # boss (radius 6 + 1 mm)
        z_a = -58.0 + 7.0 * math.hypot(1, ROOF) - (15.0 - kb) * ROOF + 0.3
        for sg in (1, -1):
            pts = [(kb, z_a), (40, z_a + (40 - kb) * ROOF), (40, z_top),
                   (hb, z_top), (hb, zs), (kb, zs)]
            pts = [(sg * a, b) for a, b in pts]
            if sg < 0:
                pts = pts[::-1]
            pl = pl.cut(prism(PB, pts, -1, T_B + 1))
    return pl


def core():
    """Core = slab between the plates  AND  A prism (only as far as A's bolt
    spaces reach)  AND  B prism (whole length)."""
    big = [(-200, -200), (200, -200), (200, 200), (-200, 200)]
    # A section (A's u-z frame): top web between A1/A6, roof, sides between
    # A2/A5, bottom above A3/A4.
    a_sec = roof_pts(web=P['WA'], z_web=30.0, half=P['HA'], z_bot=P['ZBA'])
    pa = prism(PA, a_sec, -1, A_REACH).fuse(prism(PA, big, A_REACH, 300))
    # B section (B's s-z frame): top web between B3/B4, roof, sides, then a
    # narrower keel down between B1/B2.
    wb, hb, kb, zs = P['WB'], P['HB'], P['KB'], P['ZSTEP']
    z_side = 20.4 - (hb - wb) * ROOF
    if P['ZKB'] < zs - 1.0:      # optional keel below the core
        bot = [(-hb, zs), (-kb, zs), (-kb, P['ZKB']), (kb, P['ZKB']), (kb, zs), (hb, zs)]
    else:
        bot = [(-hb, zs), (hb, zs)]
    b_sec = [(-wb, Z_BED), (-wb, 20.4), (-hb, z_side)] + bot + [
             (hb, z_side), (wb, 20.4), (wb, Z_BED)]
    pb = prism(PB, b_sec, -1, 200)
    slabA = prism(PA, [(-200, -200), (200, -200), (200, Z_BED), (-200, Z_BED)], T_A - 0.5, 200)
    slabB = prism(PB, [(-200, -200), (200, -200), (200, Z_BED), (-200, Z_BED)], T_B - 0.5, 200)
    return slabA.common(slabB).common(pa).common(pb)


def holes(shape):
    for b in BOLTS:
        p = V(*b["position"]); ax = V(*b["axis"]).normalize()
        d = D_HOLE_A if b["size"] == "M5" else D_HOLE_B
        cyl = Part.makeCylinder(d / 2, 40, p - ax * 10, ax)
        shape = shape.cut(cyl)
    return shape


def build() -> Part.Shape:
    s = plate_A().fuse(plate_B()).fuse(core())
    s = s.removeSplitter()
    s = holes(s)
    s = s.removeSplitter()
    return s


OUT = os.environ.get("CONN_OUT", os.path.join(HERE, "output.step"))
try:
    if os.path.exists(OUT):
        os.remove(OUT)
    shape = build()
    if len(shape.Solids) == 1:
        shape = shape.Solids[0].removeSplitter()
    print("solids", len(shape.Solids), "valid", shape.isValid(), "volume", shape.Volume)
    shape.exportStep(OUT)
    print("wrote", OUT)
except Exception:
    traceback.print_exc()
    if os.path.exists(OUT):
        os.remove(OUT)
    sys.exit(1)
