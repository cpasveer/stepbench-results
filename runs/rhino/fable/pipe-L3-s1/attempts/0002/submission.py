#! python3
"""pipe-L3-s1: saddle pad on the pipe + solid cranked beam + flat pad on B.

Printed lying down, pipe axis vertical (+pipe-axis side on the bed): every wall
is vertical, the B pad leans 40 deg from vertical and stands on its own foot.
"""
import json, math, os, traceback

HERE = os.path.dirname(os.path.abspath(__file__.replace("file://", "")))
OUT = os.path.join(HERE, "output.step")
LOG = os.path.join(HERE, "submission.log")

import Rhino
import Rhino.Geometry as rg

TASK = json.load(open(os.path.join(HERE, "task.json")))
POSE = TASK["poses"][0]
BOLTS = POSE["resolved"]["bolts"]
TOL = 0.001
MSG = []

# ---- design numbers -------------------------------------------------------
R_PIPE = 34.0
PAD_T = 4.0          # pad thickness under the M5 heads (grip)
PAD_HALF_X = 22.2    # pad half length along the pipe
PAD_END_DEG = 8.0    # pad runs this far past each bolt row
PLATE_T = 3.2        # B pad thickness (grip)
PL_A0, PL_A1 = -16.5, 17.0
PL_B = 16.5
BEAM_HW = 8.3        # beam half width
BEAM_H = 2 * PAD_HALF_X  # beam as tall as the pad is long: top and bottom flush
KINK_W = 59.0
A_HOLE = 5.5
B_HOLE = 4.5


def V3(c):
    return rg.Vector3d(c[0], c[1], c[2])


def col(rot, j):
    return rg.Vector3d(rot[0][j], rot[1][j], rot[2][j])


pa = POSE["placements"]["A"]
pb = POSE["placements"]["B"]
O = rg.Point3d(*pa["origin"])
Xh = col(pa["rotation"], 0); Xh.Unitize()
C = rg.Point3d(*pb["origin"])
e1 = col(pb["rotation"], 0)
e2 = col(pb["rotation"], 1)
nB = col(pb["rotation"], 2)
Vh = rg.Vector3d(e2)
MSG.append("Xh.e2 = %.3e" % (Xh * Vh))
Vh = Vh - Xh * (Xh * Vh); Vh.Unitize()
Wh = rg.Vector3d.CrossProduct(Xh, Vh)
if (C - O) * Wh < 0:
    Wh = -Wh


def P(x, w, v):
    return O + Xh * x + Wh * w + Vh * v


def PB(a, b, h):
    return C + e1 * a + e2 * b + nB * h


def frame(p):
    d = p - O
    return d * Xh, d * Wh, d * Vh


Xc, Wc, Vc = frame(C)
X_BED = PAD_HALF_X
MSG.append("C in pipe frame X,W,V = %.3f %.3f %.3f" % (Xc, Wc, Vc))
MSG.append("nB in frame = %.4f %.4f %.4f" % (nB * Xh, nB * Wh, nB * Vh))


def prism(pts, vec):
    pts = list(pts) + [pts[0]]
    crv = rg.PolylineCurve(pts)
    srf = rg.Surface.CreateExtrusion(crv, vec)
    b = srf.ToBrep()
    b.Faces.SplitKinkyFaces(0.01, True)
    b = b.CapPlanarHoles(TOL)
    if b is None or not b.IsSolid:
        raise RuntimeError("prism not solid")
    if b.SolidOrientation == rg.BrepSolidOrientation.Inward:
        b.Flip()
    return b


def cyl(base, axis, r, length):
    c = rg.Cylinder(rg.Circle(rg.Plane(base, axis), r), length)
    return c.ToBrep(True, True)


def one(res, what):
    if res is None or len(res) != 1:
        raise RuntimeError("%s gave %s pieces" % (what, None if res is None else len(res)))
    return res[0]


def build():
    # ---- pad on the pipe ------------------------------------------------
    phis = []
    for b in BOLTS:
        if b["id"].startswith("A"):
            x, w, v = frame(rg.Point3d(*b["position"]))
            phis.append(math.atan2(v, w))
    ph1, ph2 = min(phis), max(phis)
    ph0 = math.atan2(Vc, KINK_W)            # the beam meets the pad square on, along this ray
    MSG.append("bolt rows at %.2f / %.2f deg, beam ray %.2f deg" % (math.degrees(ph1), math.degrees(ph2), math.degrees(ph0)))
    Ro = R_PIPE + PAD_T
    dE = math.radians(PAD_END_DEG)

    def pol(r, ph, x):
        return P(x, r * math.cos(ph), r * math.sin(ph))

    x0 = -PAD_HALF_X
    pad_pts = [
        pol(28.0, ph1 - dE, x0),
        pol(Ro / math.cos(dE), ph1 - dE, x0),
        pol(Ro / math.cos((ph0 - ph1) / 2), (ph0 + ph1) / 2, x0),
        pol(Ro / math.cos((ph2 - ph0) / 2), (ph0 + ph2) / 2, x0),
        pol(Ro / math.cos(dE), ph2 + dE, x0),
        pol(28.0, ph2 + dE, x0),
    ]
    pad = prism(pad_pts, Xh * (2 * PAD_HALF_X))
    pipe = cyl(O - Xh * 60.0, Xh, R_PIPE, 120.0)
    pad = one(rg.Brep.CreateBooleanDifference([pad], [pipe], TOL), "pad - pipe")

    # ---- B pad with its foot (one profile, extruded across the pad) --------
    def XW(p):
        d = p - O
        return d * Xh
    E = PB(PL_A1, -PL_B, 0.0)
    F = PB(PL_A1, -PL_B, PLATE_T)
    T1 = PB(PL_A0, -PL_B, 0.0)
    T2 = PB(PL_A0, -PL_B, PLATE_T)
    G1 = E + Xh * (X_BED - XW(E))
    G2 = F + Xh * (X_BED - XW(F))
    plate = prism([T1, E, G1, G2, F, T2], e2 * (2 * PL_B))

    # ---- beam ---------------------------------------------------------------
    s0, c0 = math.sin(ph0), math.cos(ph0)
    hw = BEAM_HW
    w_end = 0.5 * (frame(E)[1] + frame(F)[1])
    r_in = R_PIPE + 0.5 * PAD_T
    xb = X_BED - BEAM_H

    def wv(w, v):
        return P(xb, w, v)
    beam_pts = [
        wv(r_in * c0 + hw * s0, r_in * s0 - hw * c0),                 # pad end, low side
        wv((c0 * (Vc - hw) + hw) / s0, Vc - hw),                      # inside of the crank
        wv(w_end, Vc - hw),
        wv(w_end, Vc + hw),
        wv((c0 * (Vc + hw) - hw) / s0, Vc + hw),                      # outside of the crank
        wv(r_in * c0 - hw * s0, r_in * s0 + hw * c0),                 # pad end, high side
    ]
    MSG.append("crank corners W = %.2f / %.2f, beam end W = %.2f" % ((c0 * (Vc - hw) + hw) / s0, (c0 * (Vc + hw) - hw) / s0, w_end))
    beam = prism(beam_pts, Xh * BEAM_H)
    # trim the beam end so that it stops inside the B pad (never reaches B)
    hc = PLATE_T    # the beam ends on the plane of the B pad's front face
    cut = prism([PB(-80, -60, hc), PB(80, -60, hc), PB(80, -60, -80), PB(-80, -60, -80)], e2 * 120.0)
    beam = one(rg.Brep.CreateBooleanDifference([beam], [cut], TOL), "beam trim")

    body = one(rg.Brep.CreateBooleanUnion([pad, beam, plate], TOL), "union")

    # ---- holes ---------------------------------------------------------------
    cutters = []
    for b in BOLTS:
        p = rg.Point3d(*b["position"]); ax = V3(b["axis"]); ax.Unitize()
        if b["id"].startswith("A"):
            cutters.append(cyl(p - ax * 3.0, ax, A_HOLE / 2, 3.0 + PAD_T + 3.0))
        else:
            cutters.append(cyl(p - ax * 1.5, ax, B_HOLE / 2, 1.5 + PLATE_T + 1.5))
    for c in cutters:
        body = one(rg.Brep.CreateBooleanDifference([body], [c], TOL), "hole")
    body.MergeCoplanarFaces(TOL)
    body.Compact()
    return body


def describe(b):
    MSG.append("valid=%s solid=%s faces=%d edges=%d" % (b.IsValid, b.IsSolid, b.Faces.Count, b.Edges.Count))
    vmp = rg.VolumeMassProperties.Compute(b)
    MSG.append("volume = %.0f mm3" % vmp.Volume)
    el = sorted(e.GetLength() for e in b.Edges)
    MSG.append("shortest edges: " + ", ".join("%.3f" % x for x in el[:6]))
    ar = sorted(rg.AreaMassProperties.Compute(f).Area for f in b.Faces)
    MSG.append("smallest faces: " + ", ".join("%.2f" % x for x in ar[:6]))
    kinds = {}
    for f in b.Faces:
        s = f.UnderlyingSurface()
        k = "plane" if f.IsPlanar(TOL) else ("cyl" if f.IsCylinder(TOL) else type(s).__name__)
        kinds[k] = kinds.get(k, 0) + 1
    MSG.append("faces by kind: %s" % kinds)
    for f in b.Faces:
        amp = rg.AreaMassProperties.Compute(f)
        c = amp.Centroid
        x, w, v = frame(c)
        ok, u, vv = f.ClosestPoint(c)
        nrm = f.NormalAt(u, vv)
        if f.OrientationIsReversed:
            nrm = -nrm
        MSG.append("  f%-2d %-5s area %8.2f  cen X %7.2f W %7.2f V %7.2f  n(X,W,V) %5.2f %5.2f %5.2f" % (
            f.FaceIndex, "plane" if f.IsPlanar(TOL) else "curv", amp.Area, x, w, v, nrm * Xh, nrm * Wh, nrm * Vh))


def main():
    if os.path.exists(OUT):
        os.remove(OUT)
    doc = Rhino.RhinoDoc.CreateHeadless(None)
    try:
        doc.ModelUnitSystem = Rhino.UnitSystem.Millimeters
        doc.ModelAbsoluteTolerance = TOL
        body = build()
        describe(body)
        doc.Objects.AddBrep(body)
        if not doc.Export(OUT):
            raise RuntimeError("Rhino could not export output.step")
    finally:
        doc.Dispose()
    open(LOG, "w").write("ok: wrote output.step\n" + "\n".join(MSG) + "\n")


try:
    main()
except Exception:
    err = traceback.format_exc().replace(HERE, ".")
    open(LOG, "w").write("error:\n" + err + "\n".join(MSG) + "\n")
    if os.path.exists(OUT):
        os.remove(OUT)
