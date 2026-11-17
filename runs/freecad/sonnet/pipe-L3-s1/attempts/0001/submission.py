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

# ---- parameters (mm)
R_PIPE = 34.0
T_PAD = 3.95          # thickness under M5 heads (grip)
R_OUT = R_PIPE + T_PAD
BAR_W = 10.0          # width of the ring bars on the pipe
T_B = 3.6             # plate thickness on B
POST_W = 15.0
POST_L = 38.0         # straight length of post along the face normal
BOSS_R = 5.0
ARM_W = 8.0
STRUT_TH = 8.0        # how far the strut dips into the pipe (cut away)
LAND_X, LAND_TH = 17.5, 10.0


def vec(a):
    return V(*a)


# pipe frame
_PL = TASK["poses"][0]["placements"]["A"]
_R = _PL["rotation"]
AX = V(_R[0][0], _R[1][0], _R[2][0]); AX.normalize()
C = vec(_PL["origin"])
TOP = V(_R[0][2], _R[1][2], _R[2][2]); TOP.normalize()
SIDE = AX.cross(TOP)


def pipe_to_world(x, y, z):
    return C + AX * x + TOP * y + SIDE * z


def pipe_matrix():
    m = App.Matrix(AX.x, TOP.x, SIDE.x, C.x,
                   AX.y, TOP.y, SIDE.y, C.y,
                   AX.z, TOP.z, SIDE.z, C.z,
                   0, 0, 0, 1)
    return m


def pipe_local(shape):
    s = shape.copy()
    s.transformShape(pipe_matrix())
    return s


def wedge(t0, t1, x0, x1):
    """pie prism in pipe-local coords between angles t0..t1 (deg from +y toward +z), x0..x1"""
    pts = []
    pts.append(V(x0, 0, 0))
    for t in (t0, t1):
        r = 60.0
        pts.append(V(x0, r * math.cos(math.radians(t)), r * math.sin(math.radians(t))))
    pts.append(pts[0])
    w = Part.Wire(Part.makePolygon(pts))
    f = Part.Face(w)
    return f.extrude(V(x1 - x0, 0, 0))


def annulus(x0, x1):
    o = Part.makeCylinder(R_OUT, x1 - x0, V(x0, 0, 0), V(1, 0, 0))
    i = Part.makeCylinder(R_PIPE, x1 - x0, V(x0, 0, 0), V(1, 0, 0))
    return o.cut(i)


def build():
    th_pad = 35.0
    dth = math.degrees(math.asin((BAR_W / 2) / (R_PIPE + T_PAD / 2)))
    xo = 17.5
    ann = annulus(-xo - BAR_W / 2, xo + BAR_W / 2)
    bars = []
    # cross bars (x = +-17.5), spanning angles -(35+dth)..(35+dth)
    for x in (-xo, xo):
        w = wedge(-(th_pad + dth), th_pad + dth, x - BAR_W / 2, x + BAR_W / 2)
        bars.append(ann.common(w))
    # long bars at the two pad rows
    for sgn in (-1, 1):
        w = wedge(sgn * th_pad - dth, sgn * th_pad + dth, -xo - BAR_W / 2, xo + BAR_W / 2)
        bars.append(ann.common(w))
    ring = bars[0]
    for b in bars[1:]:
        ring = ring.fuse(b)
    ring = pipe_local(ring)

    # ---- B side, in world coordinates
    n = vec(FACE["normal"]); n.normalize()
    ea = V(0, 0, 1)
    eb = n.cross(ea)
    o = vec(FACE["origin"])

    def bm(p):  # B-local -> world matrix columns
        return o + ea * p[0] + eb * p[1] + n * p[2]

    m = App.Matrix(ea.x, eb.x, n.x, o.x, ea.y, eb.y, n.y, o.y, ea.z, eb.z, n.z, o.z, 0, 0, 0, 1)

    def to_world(s):
        s = s.copy(); s.transformShape(m); return s

    parts = []
    for sx in (-12.5, 12.5):
        for sy in (-12.5, 12.5):
            parts.append(Part.makeCylinder(BOSS_R, T_B, V(sx, sy, 0), V(0, 0, 1)))
    # diagonal arms
    L = 12.5 * math.sqrt(2)
    for ang in (45, 135):
        bx = Part.makeBox(2 * L, ARM_W, T_B, V(-L, -ARM_W / 2, 0))
        bx.rotate(V(0, 0, 0), V(0, 0, 1), ang)
        parts.append(bx)
    post = Part.makeBox(POST_W, POST_W, POST_L, V(-POST_W / 2, -POST_W / 2, 0))
    parts.append(post)
    plate = parts[0]
    for p in parts[1:]:
        plate = plate.fuse(p)
    plate = to_world(plate)

    # strut: oblique prism of the post's square section
    P1 = o + n * POST_L
    N = pipe_to_world(LAND_X, R_PIPE * math.cos(math.radians(LAND_TH)),
                      R_PIPE * math.sin(math.radians(LAND_TH)))
    d = N - P1; dist = d.Length; d.normalize()
    h = POST_W / 2
    pts = [P1 + ea * sx * h + eb * sy * h for sx, sy in ((-1, -1), (1, -1), (1, 1), (-1, 1))]
    pts.append(pts[0])
    face = Part.Face(Part.Wire(Part.makePolygon(pts)))
    strut = face.extrude(d * (dist + STRUT_TH))

    body = plate.fuse(strut).fuse(ring)
    # pipe surface
    pipe = Part.makeCylinder(R_PIPE, 400, C - AX * 200, AX)
    body = body.cut(pipe)
    # holes
    for k, b in BOLTS.items():
        ax = vec(b["axis"]); ax.normalize()
        p = vec(b["position"])
        dia = 5.5 if k.startswith("A") else 4.5
        cyl = Part.makeCylinder(dia / 2, 20, p - ax * 8, ax)
        body = body.cut(cyl)
    body = body.removeSplitter()
    return body


OUT = os.path.join(HERE, "output.step")
try:
    if os.path.exists(OUT):
        os.remove(OUT)
    shape = build()
    sol = shape.Solids
    print("solids", len(sol), "vol", shape.Volume, "valid", shape.isValid())
    if len(sol) == 1:
        shape = sol[0]
    # self-check against keep-outs and fixed parts
    for nm in ("fixed_A.step", "fixed_B.step"):
        f = Part.read(os.path.join(HERE, nm))
        print(nm, "overlap", shape.common(f).Volume)
    for k, b in BOLTS.items():
        ax = vec(b["axis"]); ax.normalize(); p = vec(b["position"])
        if k.startswith("A"):
            g, hh, r = T_PAD, 5.0, 4.25
        else:
            g, hh, r = T_B, 4.0, 3.5
        keep = Part.makeCylinder(r + 0.5, hh + 31 + 1, p + ax * g, ax)
        print(k, "keepout overlap", round(shape.common(keep).Volume, 4))
    shape.exportStep(OUT)
    print("wrote", OUT)
except Exception:
    traceback.print_exc()
    if os.path.exists(OUT):
        os.remove(OUT)
    sys.exit(1)
