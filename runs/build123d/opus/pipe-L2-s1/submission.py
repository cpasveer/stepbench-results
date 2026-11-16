"""Pipe-to-square-mount bracket (task pipe-L2-s1), build123d.

Concept
-------
* A saddle pad hugs the 68 mm pipe (bore R34) over +-50 deg and 56 mm of its
  length.  Its outside is two flats, one per bolt row, each square to its
  bolt axes, so the M5 heads bear on true flat spot faces (grip 8 mm).
* A 7 mm flange plate sits on B's square face (seat 1 mm inside its edge),
  M4 heads on its front face.
* A web joins pad and plate.  It is a prism along the pipe axis, which is
  the print direction: the part prints standing on the pad's end face, the
  web's walls are vertical, the saddle and the bolt flats are vertical, and
  the B seat faces down at ~50 deg (self-supporting).  Below the flange's
  lower edge the web is closed by a 47.5 deg face, and the flange's lower
  edge is drafted, so nothing overhangs more than 45 deg.

All modelling is done in the pipe's local frame (x = pipe axis, z = towards
the middle of the bolt pattern) and moved to world coordinates at the end.
"""
import json
import math
import pathlib

import numpy as np
from build123d import (Box, Cylinder, Location, Plane, Polyline, Pos,
                       Rot, Part, Vector, export_step, extrude, make_face,
                       Line, ThreePointArc, Wire, Face, Solid, Axis, fillet)

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
POSE = TASK["poses"][0]
BOLTS = POSE["resolved"]["bolts"]

RA = np.array(POSE["placements"]["A"]["rotation"])
OA = np.array(POSE["placements"]["A"]["origin"])
RB = np.array(POSE["placements"]["B"]["rotation"])
OB = np.array(POSE["placements"]["B"]["origin"])

# ---- design parameters --------------------------------------------------
R_PIPE = 34.0
PAD_T = 8.0            # pad thickness under the M5 heads (grip)
PAD_HALF_ANG = 50.0    # saddle wrap, each side of the pattern centre
PAD_X = (-28.0, 28.0)  # pad extent along the pipe axis
FL_T = 7.0             # flange thickness (grip of the M4)
FL_HALF = 17.5         # flange seat half size (face is 19, 1 mm set back)
FL_LOW = -17.0         # seat's lower edge (u), 2 mm inside B's edge
WEB_TOP = -2.0         # top of the web along the pipe axis
WEB_HALF_V = 7.0       # web half width at the flange (between M4 heads)
CLIP_ALPHA = 46.0      # closing face below the seat, angle from "down"
WEB_MID_U = -36.0      # web reaches full narrow width here (flange coords)
WEB_TOP_U = -4.0       # web's end inside the flange
WEB_FILLET = 8.0


def loc(p):
    return RA.T @ (np.asarray(p, float) - OA)


def vec(d):
    return RA.T @ np.asarray(d, float)


V = lambda a: Vector(*[float(c) for c in a])

# B frame in pipe-local coordinates
F = loc(OB)
XB = vec(RB[:, 0])
YB = vec(RB[:, 1])
NB = vec(RB[:, 2])
assert np.allclose(np.cross(XB, YB), NB, atol=1e-6)


def rad(t_deg):
    t = math.radians(t_deg)
    return np.array([-math.sin(t), math.cos(t)])  # (y, z); +35 deg = A1/A2 row


def yz_plane(x0):
    # 2D coords (a, b) = local (y, z); normal = +x
    return Plane(origin=(x0, 0, 0), x_dir=(0, 1, 0), z_dir=(1, 0, 0))


def pad():
    r_out = R_PIPE + PAD_T
    a = PAD_HALF_ANG
    r_end = r_out / math.cos(math.radians(a - 35.0))
    ridge = r_out / math.cos(math.radians(35.0))
    p1 = R_PIPE * rad(a)
    p2 = r_end * rad(a)
    p3 = np.array([0.0, ridge])
    p4 = r_end * rad(-a)
    p5 = R_PIPE * rad(-a)
    pm = R_PIPE * rad(0)
    pts = [tuple(p) for p in (p1, p2, p3, p4, p5)]
    pl = yz_plane(PAD_X[0])
    edges = []
    g = lambda p: pl.from_local_coords((float(p[0]), float(p[1])))
    for q0, q1 in zip(pts[:-1], pts[1:]):
        edges.append(Line(g(q0), g(q1)))
    edges.append(ThreePointArc(g(p5), g(pm), g(p1)))
    face = Face(Wire(edges))
    return extrude(face, PAD_X[1] - PAD_X[0], dir=(1, 0, 0))


def flange_plane():
    # 2D coords (u, n); extrusion along -YB
    return Plane(origin=V(F + FL_HALF * YB), x_dir=V(XB), z_dir=V(-YB))


def bed_halfspace():
    return halfspace(np.array([PAD_X[0], 0, 0]), np.array([1.0, 0, 0]))


def clip_halfspace():
    """Closes everything below the seat's lower edge with a face at
    CLIP_ALPHA from straight down; it leaves B's face plane at the edge
    and stays in front of it, so nothing else touches B."""
    up = np.array([1.0, 0, 0])
    e_pt = plate_pt(FL_LOW, 0, 0)
    beta = math.radians(math.degrees(math.acos(float(NB @ up))) - CLIP_ALPHA)
    o = -NB * math.cos(beta) - XB * math.sin(beta)  # outward normal
    o = o / np.linalg.norm(o)
    return halfspace(e_pt, -o)


def flange():
    """Flange plate on B, carried down to the bed as a foot so that its
    lower edge is never a level edge in mid-air."""
    pl = flange_plane()
    pts = [(-80.0, 0), (FL_HALF, 0), (FL_HALF, FL_T), (-80.0, FL_T)]
    face = make_face(Polyline(*[pl.from_local_coords(p) for p in pts], close=True))
    slab = extrude(face, 2 * FL_HALF, dir=V(-YB))
    return slab & bed_halfspace() & clip_halfspace()


def halfspace(origin, normal, size=600.0):
    """Big box occupying the side of the plane the normal points to."""
    pl = Plane(origin=V(origin), z_dir=V(normal))
    return Part(Box(size, size, size).locate(Location(pl) * Pos(0, 0, size / 2)).wrapped)


def plate_pt(u, v, n):
    return F + u * XB + v * YB + n * NB


def web():
    """Prism along the print direction joining pad and flange.  Near the
    flange it is a constant-width strip (|v| <= WEB_HALF_V) that runs up
    between the M4 heads; at the pad it straddles the ridge between the
    two M5 rows."""
    yz = lambda p: (float(p[1]), float(p[2]))
    q = [(-11.0, 40.0), (11.0, 40.0),
         yz(plate_pt(WEB_MID_U, -WEB_HALF_V, 0)), yz(plate_pt(WEB_TOP_U, -WEB_HALF_V, 0)),
         yz(plate_pt(WEB_TOP_U, WEB_HALF_V, 0)), yz(plate_pt(WEB_MID_U, WEB_HALF_V, 0))]
    pl = yz_plane(PAD_X[0])
    face = make_face(Polyline(*[pl.from_local_coords(p) for p in q], close=True))
    concave = [v for v in face.vertices()
               if min(abs(Vector(v) - pl.from_local_coords(q[i])) for i in (2, 5)) < 1e-6]
    face = fillet(concave, WEB_FILLET)
    prism = extrude(face, WEB_TOP - PAD_X[0], dir=(1, 0, 0))
    return prism & halfspace(F, NB) & clip_halfspace()


def build():
    body = pad() + flange() + web()
    for b in BOLTS:
        p = loc(b["position"])
        a = vec(b["axis"])
        d = 5.5 if b["size"] == "M5" else 4.5
        length = 40.0
        c = Cylinder(d / 2, length).locate(
            Location(Plane(origin=V(p - 3.0 * a), z_dir=V(a))) * Pos(0, 0, length / 2))
        body = body - c
    world = Location(Plane(origin=V(OA), x_dir=V(RA[:, 0]), z_dir=V(RA[:, 2])))
    return body.moved(world)


if __name__ == "__main__":
    part = build()
    sol = part.solids()
    print("solids", len(sol), "volume", part.volume)
    export_step(part, str(HERE / "output.step"))
    print("wrote", HERE / "output.step")
