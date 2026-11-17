"""Pipe-to-square-mount connector, FreeCAD.  Run: freecadcmd submission.py"""
import json, math, os, sys, traceback
import FreeCAD as App
import Part

try:
    HERE = os.path.dirname(os.path.abspath(__file__))
except NameError:
    HERE = os.getcwd()
TASK = json.load(open(os.path.join(HERE, "task.json")))
POSE = TASK["poses"][0]["resolved"]
V = App.Vector
BOLTS = {b["id"]: b for b in POSE["bolts"]}
FACE = POSE["mounting_faces"][0]
PRM = dict(
    T_PAD=4.0, T_ARCH=3.5, BAR_W=9.0, T_B=3.4, BOSS_H=5.4, ARM_W=7.4,
    PLATE_FULL=0, ARCH_TH=38.0, ARCH_W=12.0, FL_T=2.5, WING_TH=70.0, LEAN0=24.0, SA=0.0, BEAM_T=18.0, PSI=48.0, NECK=40.0, SROOT=1.5, BEAM_A=13.0, BEAM_B=7.65, LAND_X=-9.5, LAND_TH=0.0, SB=0.0, DIP=8.0, BED_X=-22.5,
)
PRM.update(json.loads(os.environ.get("PRM", "{}")))
globals().update(PRM)
R_PIPE = 34.0


def vec(a):
    return V(*a)


_PL = TASK["poses"][0]["placements"]["A"]
_R = _PL["rotation"]
AX = V(_R[0][0], _R[1][0], _R[2][0]); AX.normalize()
C = vec(_PL["origin"])
TOP = V(_R[0][2], _R[1][2], _R[2][2]); TOP.normalize()
SIDE = AX.cross(TOP)
PM = App.Matrix(AX.x, TOP.x, SIDE.x, C.x, AX.y, TOP.y, SIDE.y, C.y, AX.z, TOP.z, SIDE.z, C.z, 0, 0, 0, 1)


def pipe_to_world(x, y, z):
    return C + AX * x + TOP * y + SIDE * z


def to_pipe(s):
    s = s.copy(); s.transformShape(PM); return s


def prism(pts, vec_):
    pts = list(pts) + [pts[0]]
    return Part.Face(Part.Wire(Part.makePolygon(pts))).extrude(vec_)


def build():
    # ---------------- A side (pipe-local coordinates, then to world)
    th = 35.0
    xs = 22.5
    parts = []
    for sgn in (-1, 1):
        bx = Part.makeBox(2 * xs, BAR_W, 8.0 + 0.0, V(-xs, -BAR_W / 2, R_PIPE - 4.0))
        # top plane at radial R_PIPE + T_PAD
        bx = Part.makeBox(2 * xs, BAR_W, T_PAD + 4.0, V(-xs, -BAR_W / 2, 0))
        bx.translate(V(0, 0, 0))
        # box: x, tangential(z), radial(y); build with radial along y
        bx = Part.makeBox(2 * xs, T_PAD + 4.0, BAR_W, V(-xs, R_PIPE - 4.0, -BAR_W / 2))
        bx.rotate(V(0, 0, 0), V(1, 0, 0), sgn * th)
        parts.append(bx)
    # arch at the bottom of the pipe-axis print: a band across the top of the pipe
    def band(length, half_deg, r_out):
        ann = Part.makeCylinder(r_out, length, V(BED_X, 0, 0), V(1, 0, 0)).cut(
            Part.makeCylinder(R_PIPE - 1, length, V(BED_X, 0, 0), V(1, 0, 0)))
        pts = [V(BED_X - 1, 0, 0)]
        for t in (-half_deg, 0.0, half_deg):
            pts.append(V(BED_X - 1, 60 * math.cos(math.radians(t)), 60 * math.sin(math.radians(t))))
        return ann.common(prism(pts, V(length + 2.0, 0, 0)))
    parts.append(band(ARCH_W, ARCH_TH, R_PIPE + T_ARCH))
    if FL_T > 0:
        parts.append(band(FL_T, WING_TH, R_PIPE + T_ARCH))
    a_side = parts[0]
    for p in parts[1:]:
        a_side = a_side.fuse(p)
    a_side = to_pipe(a_side)

    # ---------------- B side (world coordinates)
    n = vec(FACE["normal"]); n.normalize()
    ea = V(0, 0, 1)
    eb = n.cross(ea)
    o = vec(FACE["origin"])
    m = App.Matrix(ea.x, eb.x, n.x, o.x, ea.y, eb.y, n.y, o.y, ea.z, eb.z, n.z, o.z, 0, 0, 0, 1)

    def to_world(s):
        s = s.copy(); s.transformShape(m); return s

    pl = []
    AW = ARM_W
    L = 12.5 * math.sqrt(2) + AW / 2
    for ang in (45, 135):
        bx = Part.makeBox(2 * L, AW, T_B, V(-L, -AW / 2, 0))
        bx.rotate(V(0, 0, 0), V(0, 0, 1), ang)
        pl.append(bx)
    if PLATE_FULL:
        pl.append(Part.makeBox(35, 35, T_B, V(-17.5, -17.5, 0)))
    plate = pl[0]
    for p in pl[1:]:
        plate = plate.fuse(p)
    plate = to_world(plate)

    # beam from the plate to the pipe.  Neck: straight, in the plane (n, ea) through
    # o + SA*ea + SB*eb; then a sheared taper to the landing on the pipe.
    Q = o + eb * SB + ea * SA
    N = pipe_to_world(LAND_X, R_PIPE * math.cos(math.radians(LAND_TH)), R_PIPE * math.sin(math.radians(LAND_TH)))
    d = n * math.cos(math.radians(LEAN0)) - ea * math.sin(math.radians(LEAN0))
    d.normalize()
    q = d.cross(eb); q.normalize()
    if q.dot(AX) < 0:
        q = q * -1.0
    start = Q - d * 6.0
    t_neck = 6.0 + NECK / d.dot(n)
    Pn = start + d * t_neck
    d1 = N - Pn; d1.normalize()
    c_end = N + d1 * DIP
    cb = math.tan(math.radians(PSI))   # chamfer rise per half width

    def poly(sc):
        B = BEAM_B * sc; T = BEAM_T * sc
        return [(-B, T), (B, T), (B, 0.0), (0.0, -cb * B), (-B, 0.0)]

    def section(c0, sc):
        return [c0 + eb * x + q * y for x, y in poly(sc)]

    def frustum(c0, sc0, c1, sc1):
        s0 = section(c0, sc0); s1 = section(c1, sc1)
        n_ = len(s0)
        faces = [Part.Face(Part.Wire(Part.makePolygon(s0 + [s0[0]]))), Part.Face(Part.Wire(Part.makePolygon(s1 + [s1[0]])))]
        for i in range(n_):
            j = (i + 1) % n_
            faces.append(Part.Face(Part.Wire(Part.makePolygon([s0[i], s0[j], s1[j], s1[i], s0[i]]))))
        sol = Part.Solid(Part.Shell(faces))
        if sol.Volume < 0:
            sol.reverse()
        return sol
    beam = frustum(start, 1.0, Pn, 1.0)
    beam = beam.fuse(frustum(Pn, 1.0, c_end, SROOT))
    # keep the beam on the front side of the face
    half = Part.makeBox(300, 300, 300, o - ea * 150 - eb * 150, V(0, 0, 1))  # placeholder
    big = Part.makeBox(400, 400, 200, V(-200, -200, 0))
    big = to_world(big)
    beam = beam.common(big)

    for nm_, sh_ in (('a_side', a_side), ('plate', plate), ('beam', beam)):
        print('part', nm_, 'vol', round(sh_.Volume), 'area', round(sh_.Area))
    body = a_side.fuse(plate).fuse(beam)
    pipe = Part.makeCylinder(R_PIPE, 400, C - AX * 200, AX)
    body = body.cut(pipe)
    for k, b in BOLTS.items():
        ax = vec(b["axis"]); ax.normalize()
        p = vec(b["position"])
        dia = 5.5 if k.startswith("A") else 4.5
        body = body.cut(Part.makeCylinder(dia / 2, 20, p - ax * 8, ax))
    body = body.removeSplitter()
    return body


OUT = os.environ.get("OUTSTEP", os.path.join(HERE, "output.step"))
try:
    if os.path.exists(OUT):
        os.remove(OUT)
    shape = build()
    sol = shape.Solids
    print("solids", len(sol), "vol", round(shape.Volume, 1), "area", round(shape.Area, 1), "cost", round(0.2 * shape.Volume + 0.96 * shape.Area, 0), "valid", shape.isValid())
    if len(sol) == 1:
        shape = sol[0]
    for nm in ("fixed_A.step", "fixed_B.step"):
        f = Part.read(os.path.join(HERE, nm))
        print(nm, "overlap", round(shape.common(f).Volume, 4))
    for k, b in BOLTS.items():
        ax = vec(b["axis"]); ax.normalize(); p = vec(b["position"])
        if k.startswith("A"):
            g, hh, r = T_PAD, 5.0, 4.25
        else:
            g, hh, r = T_B, 4.0, 3.5
        keep = Part.makeCylinder(r + 0.5, hh + 1.0, p + ax * g, ax).fuse(
            Part.makeCylinder(r, hh + 30.0, p + ax * g, ax))
        print(k, "keepout overlap", round(shape.common(keep).Volume, 4))
    shape.exportStep(OUT)
    print("wrote", OUT)
except Exception:
    traceback.print_exc()
    if os.path.exists(OUT):
        os.remove(OUT)
    sys.exit(1)
