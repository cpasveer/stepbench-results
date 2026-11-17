#! python3
"""Connecting bracket between pipe A (68 mm, 4x M5 radial) and square mount B (4x M4).

Design: a hollow 18 mm square arm (1.2 mm walls) rises from the centre of mount B
for 40 mm (clear of the M4 heads), then runs across to the pipe, where it ends in a
foot. Four bolt pads (5 mm grip, 10 mm dia) sit on the pipe and on the mount, joined
to the arm by bars/spokes. Pipe and clearances are cut exactly, so seats are
conforming. Self-checks (head zones, tool access, key swing) are logged.
"""
import json
import math
import os
import traceback

HERE = os.path.dirname(os.path.abspath(__file__.replace("file://", "")))
OUT = os.path.join(HERE, "output.step")
LOG = os.path.join(HERE, "submission.log")

import Rhino
import Rhino.Geometry as rg

TASK = json.load(open(os.path.join(HERE, "task.json")))
POSE = TASK["poses"][0]["resolved"]
BOLTS = {b["id"]: b for b in POSE["bolts"]}
FACE = POSE["mounting_faces"][0]

TOL = 0.01
GRIP = 5.0
PIPE_P0 = (21.0, 136.5, -7.0)
PIPE_R = 34.0
A_IDS = ["A1", "A2", "A3", "A4"]
B_IDS = ["B1", "B2", "B3", "B4"]
LOGLINES = []


def say(s):
    LOGLINES.append(str(s))


# ---------- plain vector helpers ----------
def add(a, b):
    return (a[0] + b[0], a[1] + b[1], a[2] + b[2])


def sub(a, b):
    return (a[0] - b[0], a[1] - b[1], a[2] - b[2])


def mul(a, k):
    return (a[0] * k, a[1] * k, a[2] * k)


def dot(a, b):
    return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]


def cross(a, b):
    return (a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0])


def norm(a):
    return math.sqrt(dot(a, a))


def unit(a):
    l = norm(a)
    return (a[0] / l, a[1] / l, a[2] / l)


def pt(a):
    return rg.Point3d(a[0], a[1], a[2])


def vc(a):
    return rg.Vector3d(a[0], a[1], a[2])


def perp(a):
    e = (1.0, 0.0, 0.0) if abs(a[0]) < 0.9 else (0.0, 1.0, 0.0)
    return unit(cross(a, e))


# ---------- pipe frame ----------
D = unit((0.2487956429525063, 0.5900653629615055, 0.7680648380709507))
P0 = PIPE_P0


def bpos(i):
    return tuple(BOLTS[i]["position"])


def baxis(i):
    return unit(tuple(BOLTS[i]["axis"]))


M = unit(add(baxis("A1"), baxis("A3")))  # radial direction at the middle of the bolt pattern
E = add(P0, mul(M, 36.0))  # arm foot end, inside the pipe's top zone

# ---------- mount frame ----------
C = tuple(FACE["origin"])
N = unit(tuple(FACE["normal"]))
U = unit((0.0, 0.0, 1.0))
V = cross(N, U)


def uv_s(p):
    w = sub(p, C)
    return dot(w, U), dot(w, V), dot(w, N)


def world(u, v, s):
    return add(C, add(add(mul(U, u), mul(V, v)), mul(N, s)))


# ---------- brep helpers ----------
def cyl(origin, axis, r, h):
    return rg.Cylinder(rg.Circle(rg.Plane(pt(origin), vc(axis)), r), h).ToBrep(True, True)


def box(origin, xa, ya, xr, yr, zr):
    pl = rg.Plane(pt(origin), vc(xa), vc(ya))
    return rg.Box(pl, rg.Interval(xr[0], xr[1]), rg.Interval(yr[0], yr[1]), rg.Interval(zr[0], zr[1])).ToBrep()


def slab_s(xa, ya, xr, yr, s_lo, s_hi):
    """Box on the mount frame: offset along N from s_lo to s_hi."""
    z = cross(xa, ya)
    zr = (s_lo, s_hi) if dot(z, N) >= 0 else (-s_hi, -s_lo)
    return box(C, xa, ya, xr, yr, zr)


def union(lst):
    return list(rg.Brep.CreateBooleanUnion(lst, TOL) or [])


def diff(a, b):
    return list(rg.Brep.CreateBooleanDifference(a, b, TOL) or [])


def inter(a, b):
    return list(rg.Brep.CreateBooleanIntersection(a, b, TOL) or [])


def vol(lst):
    return sum(rg.VolumeMassProperties.Compute(b).Volume for b in lst)


def pipe_solid(r):
    return cyl(sub(P0, mul(D, 160.0)), D, r, 320.0)


def radial_from_pipe(p):
    w = sub(p, P0)
    return norm(sub(w, mul(D, dot(w, D))))


# ---------- geometry of the arm ----------
NECK_TOP = 48.0  # top of the vertical neck (clears the M4 head access zones)
S0 = add(C, mul(N, NECK_TOP))
DIR = unit(sub(E, S0))
LS = norm(sub(E, S0))
YS = unit(sub(V, mul(DIR, dot(V, DIR))))


def build_parts():
    parts = []
    # vertical neck on mount B, 18 mm square, s = 0..40
    neck_o = slab_s(U, V, (-9.0, 9.0), (-9.0, 9.0), 0.0, NECK_TOP)
    neck_c = slab_s(U, V, (-7.8, 7.8), (-7.8, 7.8), -1.0, NECK_TOP + 1.0)
    # sloped arm from neck top to the pipe foot
    slope_o = box(sub(S0, mul(DIR, 6.0)), DIR, YS, (0.0, LS + 6.0), (-9.0, 9.0), (-9.0, 9.0))
    slope_c = box(sub(S0, mul(DIR, 4.0)), DIR, YS, (0.0, LS - 4.0), (-7.8, 7.8), (-7.8, 7.8))
    parts += [neck_o, slope_o]
    cavities = [neck_c, slope_c]

    # mount B: two diagonal bars through the corner pads
    s2 = math.sqrt(0.5)
    bar1 = slab_s(mul(add(U, V), s2), mul(sub(U, V), s2), (-17.68, 17.68), (-2.5, 2.5), 0.0, GRIP)
    bar2 = slab_s(mul(sub(U, V), s2), mul(add(U, V), s2), (-17.68, 17.68), (-2.5, 2.5), 0.0, GRIP)
    parts += [bar1, bar2]
    holes = []
    for i in B_IDS:
        u, v, _ = uv_s(bpos(i))
        parts.append(cyl(world(u, v, 0.0), N, 5.0, GRIP))
        holes.append(cyl(sub(bpos(i), mul(N, 3.0)), N, 2.25, 15.0))

    # pipe A: pads on the pipe, spokes from the arm foot
    lim = cyl(sub(P0, mul(D, 160.0)), D, 37.5, 320.0)
    for i in A_IDS:
        a = baxis(i)
        p = bpos(i)
        parts.append(cyl(sub(p, mul(a, 7.0)), a, 5.0, 12.0))
        holes.append(cyl(sub(p, mul(a, 10.0)), a, 2.75, 20.0))
        x = unit(sub(p, E))
        o = sub(E, mul(DIR, 5.0))
        ln = norm(sub(p, o))
        y = unit(cross(M, x))
        sp = box(o, x, y, (0.0, ln), (-2.5, 2.5), (-6.0, 6.0))
        parts += inter([sp], [lim])
    return parts, cavities, holes


def build_body():
    parts, cavities, holes = build_parts()
    body = union(parts)
    say("union parts -> %d brep(s), vol %.1f" % (len(body), vol(body)))
    for k, cb in enumerate(cavities):
        say("cavity %d valid %s solid %s vol %.1f" % (k, cb.IsValid, cb.IsSolid, rg.VolumeMassProperties.Compute(cb).Volume))
    cav = union(cavities)
    say("cavity union -> %d brep(s)" % len(cav))
    body = diff(body, cav)
    say("minus cavities -> %d brep(s), vol %.1f" % (len(body), vol(body)))
    cap_b = slab_s(U, V, (-9.0, 9.0), (-9.0, 9.0), 0.0, 5.0)
    body = union(body + [cap_b])
    say("with end caps -> %d brep(s), vol %.1f" % (len(body), vol(body)))
    body = diff(body, [pipe_solid(PIPE_R)])
    say("minus pipe -> %d brep(s), vol %.1f" % (len(body), vol(body)))
    for k, b in enumerate(body):
        c = rg.VolumeMassProperties.Compute(b).Centroid
        say("  piece %d vol %.1f centroid (%.1f, %.1f, %.1f)" % (k, rg.VolumeMassProperties.Compute(b).Volume, c.X, c.Y, c.Z))
    body = diff(body, holes)
    say("minus holes -> %d brep(s), vol %.1f" % (len(body), vol(body)))
    for b in body:
        b.MergeCoplanarFaces(TOL)
    return body


# ---------- self checks ----------
HEAD_TOP = {}


def check_heads(body):
    for i in A_IDS + B_IDS:
        a = baxis(i)
        hr = BOLTS[i]["head_diameter"] / 2.0
        hh = BOLTS[i]["head_height"]
        hb = add(bpos(i), mul(a, GRIP))
        zone = cyl(hb, a, hr + 0.5, hh + 1.0)
        tool = cyl(add(hb, mul(a, hh)), a, hr, 30.0)
        zv = vol(inter(body, [zone]))
        tv = vol(inter(body, [tool]))
        say("%s head zone overlap %.4f mm3, tool access overlap %.4f mm3" % (i, zv, tv))


def key_check(body, mesh, ids, m_size):
    leg_r = 2.0 if m_size == "M5" else 1.5
    leg_len = 70.0 if m_size == "M5" else 63.0
    for i in ids:
        a = baxis(i)
        hr = BOLTS[i]["head_diameter"] / 2.0
        hh = BOLTS[i]["head_height"]
        hb = add(bpos(i), mul(a, GRIP))
        ht = add(hb, mul(a, hh))
        c0 = add(ht, mul(a, leg_r))
        free = []
        for ang in range(0, 360, 6):
            th = math.radians(ang)
            e1 = perp(a)
            e2 = cross(a, e1)
            w = add(mul(e1, math.cos(th)), mul(e2, math.sin(th)))
            sv = cross(w, a)
            hit = False
            for k in range(0, 29):
                t = k * leg_len / 28.0
                q = add(c0, mul(w, t))
                pts = [q, add(q, mul(a, leg_r)), sub(q, mul(a, leg_r)), add(q, mul(sv, leg_r)), sub(q, mul(sv, leg_r))]
                for p in pts:
                    if mesh.IsPointInside(pt(p), 0.001, False):
                        hit = True
                        break
                    if radial_from_pipe(p) < PIPE_R + leg_r * 0.5:
                        hit = True
                        break
                    for j in A_IDS + B_IDS:
                        if j == i:
                            continue
                        aj = baxis(j)
                        hbj = add(bpos(j), mul(aj, GRIP))
                        wj = sub(p, hbj)
                        ax = dot(wj, aj)
                        rad = norm(sub(wj, mul(aj, ax)))
                        hrj = BOLTS[j]["head_diameter"] / 2.0
                        hhj = BOLTS[j]["head_height"]
                        if -leg_r <= ax <= hhj + leg_r and rad < hrj + leg_r:
                            hit = True
                            break
                    if hit:
                        break
                if hit:
                    break
            free.append(not hit)
        # longest circular run of free angles
        ring = free + free
        best, run, start_best = 0, 0, 0
        for k in range(len(ring)):
            if ring[k]:
                run += 1
                if run > best:
                    best = run
                    start_best = k - run + 1
            else:
                run = 0
        best = min(best, len(free))
        span = best * 6
        say("%s key swing: longest free arc %d deg (need 60), starting at %d deg" % (i, span, (start_best * 6) % 360))


def check_pipe_and_mount(body):
    for i in A_IDS:
        say("%s radius from pipe axis %.3f (34 expected)" % (i, radial_from_pipe(bpos(i))))
    for i in B_IDS:
        u, v, s = uv_s(bpos(i))
        say("%s mount local (%.2f, %.2f, %.3f)" % (i, u, v, s))


def main():
    if os.path.exists(OUT):
        os.remove(OUT)
    doc = Rhino.RhinoDoc.CreateHeadless(None)
    try:
        doc.ModelUnitSystem = Rhino.UnitSystem.Millimeters
        body = build_body()
        if len(body) != 1:
            raise RuntimeError("expected one solid, got %d" % len(body))
        brep = body[0]
        if not brep.IsSolid or not brep.IsValid:
            raise RuntimeError("body is not a valid closed solid")
        v = rg.VolumeMassProperties.Compute(brep).Volume
        say("volume %.1f mm3, PLA mass %.2f g (solid)" % (v, v * 1.24e-3))
        check_pipe_and_mount(body)
        check_heads(body)
        meshes = rg.Mesh.CreateFromBrep(brep, rg.MeshingParameters.Default)
        mesh = rg.Mesh()
        for mm in meshes:
            mesh.Append(mm)
        key_check(body, mesh, A_IDS, "M5")
        key_check(body, mesh, B_IDS, "M4")
        doc.Objects.AddBrep(brep)
        if not doc.Export(OUT):
            raise RuntimeError("Rhino could not export output.step")
    finally:
        doc.Dispose()
    open(LOG, "w").write("\n".join(LOGLINES) + "\nok: wrote output.step\n")


try:
    main()
except Exception:
    err = traceback.format_exc().replace(HERE, ".")
    open(LOG, "w").write("\n".join(LOGLINES) + "\nerror:\n" + err)
    if os.path.exists(OUT):
        os.remove(OUT)
