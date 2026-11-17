#! python3
"""Flange connector modelled in Rhino.

Pad A: thin flange on face A (6x M5): hub, three bars through the centre, six bosses.
Pad B: two deep ribs on face B (4x M6) with a cross bar, the 80 mm pitch carries the load.
Column: solid diamond section (vertex up and down, 20 mm side) running from face A to face B,
offset from the bolt pattern so the hex key can reach every head.

Writes output.step next to this file. Diagnostics go to submission.log
(first line starts with "ok:" on success).
"""
import json
import os
import traceback

HERE = os.path.dirname(os.path.abspath(__file__.replace("file://", "")))
OUT = os.path.join(HERE, "output.step")
LOG = os.path.join(HERE, "submission.log")

import Rhino
import Rhino.Geometry as rg

TASK = json.load(open(os.path.join(HERE, "task.json")))
POSE = TASK["poses"][0]["resolved"]
FACE = {f["part"]: f for f in POSE["mounting_faces"]}
BOLT = {b["id"]: b for b in POSE["bolts"]}

TOL = 0.01
T_A = 4.0       # pad thickness at face A (M5 grip range 3.75 - 25)
T_B = 10.0      # pad thickness at face B (M6 grip range 4.5 - 25), deep ribs for stiffness
BOSS_A = 6.0    # boss radius round each M5 hole (seat ring 2.75 to 4.25)
BOSS_B = 6.5    # boss radius round each M6 hole (seat ring 3.3 to 5.0)
HUB_A = 11.0
BAR_A = 6.0
BAR_B = 13.0
COL_OFF = (6.0, 6.0)   # column axis on face A, (along Z, along n x Z), mm from the face centre
COL_HALF = 10.0        # diamond face distance from axis (side 20 mm)
HOLE_A = 5.5           # M5 clearance (5.3 - 5.8)
HOLE_B = 6.6           # M6 clearance (6.4 - 7.0)
Z = rg.Vector3d(0, 0, 1)
DIAG = []


def V(p):
    return rg.Vector3d(p[0], p[1], p[2])


def P(p):
    return rg.Point3d(p[0], p[1], p[2])


def unit(v):
    v = rg.Vector3d(v)
    v.Unitize()
    return v


def comb(*terms):
    x = y = z = 0.0
    for t, s in terms:
        x += t.X * s
        y += t.Y * s
        z += t.Z * s
    return rg.Vector3d(x, y, z)


def dot(a, b):
    return a.X * b.X + a.Y * b.Y + a.Z * b.Z


def at(pl, a, b, c=0.0):
    return rg.Point3d(
        pl.Origin.X + pl.XAxis.X * a + pl.YAxis.X * b + pl.ZAxis.X * c,
        pl.Origin.Y + pl.XAxis.Y * a + pl.YAxis.Y * b + pl.ZAxis.Y * c,
        pl.Origin.Z + pl.XAxis.Z * a + pl.YAxis.Z * b + pl.ZAxis.Z * c,
    )


def local(pl, p):
    d = rg.Vector3d(p.X - pl.Origin.X, p.Y - pl.Origin.Y, p.Z - pl.Origin.Z)
    return (dot(d, pl.XAxis), dot(d, pl.YAxis))


def face_plane(part):
    """X = world Z, Z = outward normal (points into the space for my part), Y = normal x Z."""
    f = FACE[part]
    n = unit(V(f["normal"]))
    y = unit(rg.Vector3d.CrossProduct(n, Z))
    return rg.Plane(P(f["origin"]), Z, y)


def disk(pl, a, b, r, t):
    c = at(pl, a, b, 0.0)
    return rg.Cylinder(rg.Circle(rg.Plane(c, pl.ZAxis), r), t).ToBrep(True, True)


def bar(pl, p1, p2, w, t):
    a1, b1 = p1
    a2, b2 = p2
    da, db = a2 - a1, b2 - b1
    L = (da * da + db * db) ** 0.5
    xa = comb((pl.XAxis, da / L), (pl.YAxis, db / L))
    ya = unit(rg.Vector3d.CrossProduct(pl.ZAxis, xa))
    o = at(pl, (a1 + a2) / 2.0, (b1 + b2) / 2.0, 0.0)
    bp = rg.Plane(o, xa, ya)
    box = rg.Box(bp, rg.Interval(-L / 2.0, L / 2.0), rg.Interval(-w / 2.0, w / 2.0), rg.Interval(0.0, t))
    return box.ToBrep()


def hole(bolt, r, t):
    p = P(bolt["position"])
    ax = unit(V(bolt["axis"]))
    c = rg.Point3d(p.X - ax.X, p.Y - ax.Y, p.Z - ax.Z)
    return rg.Cylinder(rg.Circle(rg.Plane(c, ax), r), t + 3.0).ToBrep(True, True)


def vol(brep):
    return rg.VolumeMassProperties.Compute(brep).Volume


def overlap(solid, brep):
    res = rg.Brep.CreateBooleanIntersection([solid], [brep], TOL)
    if not res:
        return 0.0
    return sum(vol(b) for b in res)


def build():
    pA = face_plane("A")
    pB = face_plane("B")
    OA, OB = pA.Origin, pB.Origin
    D = rg.Vector3d(OB.X - OA.X, OB.Y - OA.Y, OB.Z - OA.Z)
    u = unit(D)  # column runs from face A to face B

    # --- column: solid diamond section, trimmed to the space between the two faces ---
    S0 = at(pA, COL_OFF[0], COL_OFF[1], 0.0)
    p1 = unit(rg.Vector3d.CrossProduct(u, Z))            # horizontal, normal to u
    p2 = unit(rg.Vector3d.CrossProduct(p1, u))           # near vertical, normal to u
    xd = unit(comb((p1, 1.0), (p2, 1.0)))                # face normals of the diamond
    yd = unit(comb((p1, 1.0), (p2, -1.0)))
    pc = rg.Plane(rg.Point3d(S0.X, S0.Y, S0.Z), xd, yd)
    col = rg.Box(pc, rg.Interval(-COL_HALF, COL_HALF), rg.Interval(-COL_HALF, COL_HALF),
                 rg.Interval(-150.0, 150.0)).ToBrep()
    hA = rg.Box(pA, rg.Interval(-300, 300), rg.Interval(-300, 300), rg.Interval(0, 300)).ToBrep()
    hB = rg.Box(pB, rg.Interval(-300, 300), rg.Interval(-300, 300), rg.Interval(0, 300)).ToBrep()
    c1 = rg.Brep.CreateBooleanIntersection([col], [hA], TOL)
    c2 = rg.Brep.CreateBooleanIntersection(list(c1), [hB], TOL)
    if not c2:
        raise RuntimeError("column trim failed")
    column = list(c2)

    # --- pad A: 6 bosses, hub, three bars through the centre ---
    bA = [BOLT["A%d" % i] for i in range(1, 7)]
    locA = [local(pA, P(b["position"])) for b in bA]
    padsA = [disk(pA, 0.0, 0.0, HUB_A, T_A)]
    for a, b in locA:
        padsA.append(disk(pA, a, b, BOSS_A, T_A))
    for i in range(6):
        for j in range(i + 1, 6):
            if abs(locA[i][0] + locA[j][0]) < 0.5 and abs(locA[i][1] + locA[j][1]) < 0.5:
                padsA.append(bar(pA, locA[i], locA[j], BAR_A, T_A))

    # --- pad B: 4 bosses, two deep ribs along the 80 mm pitch, one cross rib ---
    bB = [BOLT["B%d" % i] for i in range(1, 5)]
    locB = [local(pB, P(b["position"])) for b in bB]
    padsB = []
    for a, b in locB:
        padsB.append(disk(pB, a, b, BOSS_B, T_B))
    for i in range(4):
        for j in range(i + 1, 4):
            ai, bi = locB[i]
            aj, bj = locB[j]
            if abs(bi - bj) < 0.5 and abs(ai - aj) > 1.0:
                padsB.append(bar(pB, locB[i], locB[j], BAR_B, T_B))
    padsB.append(bar(pB, (0.0, -15.0), (0.0, 15.0), BAR_B, T_B))

    parts = column + padsA + padsB
    uni = rg.Brep.CreateBooleanUnion(parts, TOL)
    if not uni or len(uni) != 1:
        raise RuntimeError("union did not give one solid: %s" % (0 if not uni else len(uni)))
    solid = uni[0]
    solid.MergeCoplanarFaces(TOL)

    holes = [hole(b, HOLE_A / 2.0, T_A) for b in bA] + [hole(b, HOLE_B / 2.0, T_B) for b in bB]
    cut = rg.Brep.CreateBooleanDifference([solid], holes, TOL)
    if not cut or len(cut) != 1:
        raise RuntimeError("hole cut did not give one solid")
    final = cut[0]
    final.MergeCoplanarFaces(TOL)

    # --- self-checks: head clearance, hex key room, coplanar faces, short edges ---
    DIAG.append("solid valid=%s closed=%s volume=%.1f mm3 faces=%d" % (
        final.IsValid, final.IsSolid, vol(final), final.Faces.Count))
    for b in list(bA) + list(bB):
        p = P(b["position"])
        ax = unit(V(b["axis"]))
        T = T_A if b["size"] == "M5" else T_B
        hd, hh = b["head_diameter"], b["head_height"]
        tool_c = rg.Point3d(p.X + ax.X * (T + hh), p.Y + ax.Y * (T + hh), p.Z + ax.Z * (T + hh))
        tool = rg.Cylinder(rg.Circle(rg.Plane(tool_c, ax), hd / 2.0), 30.0).ToBrep(True, True)
        env_c = rg.Point3d(p.X + ax.X * T, p.Y + ax.Y * T, p.Z + ax.Z * T)
        env = rg.Cylinder(rg.Circle(rg.Plane(env_c, ax), hd / 2.0 + 0.5), hh + 1.0).ToBrep(True, True)
        DIAG.append("%s tool_overlap=%.3f head_env_overlap=%.3f" % (
            b["id"], overlap(final, tool), overlap(final, env)))
    groups = {}
    for f in final.Faces:
        amp = rg.AreaMassProperties.Compute(f)
        c = amp.Centroid
        nrm = f.NormalAt(*f.Domain(0).Mid, ) if False else None
        ok, uu, vv = f.ClosestPoint(c)
        nrm = f.NormalAt(uu, vv)
        key = (round(nrm.X, 2), round(nrm.Y, 2), round(nrm.Z, 2), round(dot(V((c.X, c.Y, c.Z)), nrm), 1))
        groups.setdefault(key, []).append(round(amp.Area, 1))
    for k, areas in groups.items():
        if len(areas) > 1:
            DIAG.append("coplanar group n=(%.2f,%.2f,%.2f) d=%.1f faces=%d areas=%s" % (k[0], k[1], k[2], k[3], len(areas), areas[:6]))
    for e in final.Edges:
        L = e.GetLength()
        if L < 0.3:
            m = e.PointAtNormalizedLength(0.5)
            DIAG.append("short edge %.3f mm at (%.2f, %.2f, %.2f)" % (L, m.X, m.Y, m.Z))
    return [final]


def main():
    if os.path.exists(OUT):
        os.remove(OUT)
    breps = build()
    doc = Rhino.RhinoDoc.CreateHeadless(None)
    try:
        doc.ModelUnitSystem = Rhino.UnitSystem.Millimeters
        for brep in breps:
            doc.Objects.AddBrep(brep)
        if not doc.Export(OUT):
            raise RuntimeError("Rhino could not export output.step")
    finally:
        doc.Dispose()
    lines = ["ok: wrote output.step"] + DIAG
    open(LOG, "w").write("\n".join(lines) + "\n")


try:
    main()
except Exception:
    err = traceback.format_exc().replace(HERE, ".")
    open(LOG, "w").write("error:\n" + err + "\n".join(DIAG) + "\n")
    if os.path.exists(OUT):
        os.remove(OUT)
