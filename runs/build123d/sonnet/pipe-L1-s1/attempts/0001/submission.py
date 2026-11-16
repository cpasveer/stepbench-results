"""Connector: two flat-topped pipe pads + saddle, one beam, one square plate on B."""
import json, pathlib
import numpy as np
from build123d import (Box, Cylinder, Part, Plane, Polygon, Vector, Location,
                       Solid, export_step)

HERE = pathlib.Path(__file__).parent
TASK = json.loads((HERE / "task.json").read_text())
POSE = TASK["poses"][0]["resolved"]
BOLTS = {b["id"]: b for b in POSE["bolts"]}
FACE = POSE["mounting_faces"][0]

PL = TASK["poses"][0]["placements"]["A"]
A0 = np.array(PL["origin"]); AD = np.array(PL["rotation"])[:, 2]  # pipe axis
AD = np.array([0.25, 0.59, 0.77]); AD /= np.linalg.norm(AD)
RP = 34.0
GA, GB = 6.0, 6.0           # grip (thickness under head)
V = lambda a: Vector(*[float(x) for x in a])


def unit(a):
    a = np.array(a, float); return a / np.linalg.norm(a)


def box(center, xd, zd, sx, sy, sz):
    pl = Plane(origin=V(center), x_dir=V(xd), z_dir=V(zd))
    return pl * Box(sx, sy, sz)


def cyl(center, zd, r, h):
    pl = Plane(origin=V(center), z_dir=V(zd))
    return pl * Cylinder(r, h)


def build() -> Part:
    ad = unit(AD)
    n1 = unit(BOLTS["A1"]["axis"]); n2 = unit(BOLTS["A3"]["axis"])
    m = unit(n1 + n2)
    parts = []
    # axis point at mid of bolt pair
    mid = (np.array(BOLTS["A1"]["position"]) + np.array(BOLTS["A2"]["position"])) / 2
    u0 = A0 + ((mid - A0) @ ad) * ad
    for n in (n1, n2):
        parts.append(box(u0 + 30 * n, ad, n, 51, 16, 20 + 0) .moved(Location()))
    # strips: top plane at distance RP+GA from axis along n
    parts = []
    for n in (n1, n2):
        h = RP + GA - 18
        t = np.cross(n, ad)
        parts.append(box(u0 + (18 + h / 2) * n, ad, n, 51, 16, h))
    # saddle between strips
    ang = np.radians(35)
    pl = Plane(origin=V(u0 - 25.5 * ad), x_dir=V(m), z_dir=V(ad))
    ys = unit(np.cross(ad, m))
    poly = Polygon((0, 0), (60 * np.cos(ang), 60 * np.sin(ang)),
                   (60 * np.cos(ang), -60 * np.sin(ang)), align=None)
    prism = Solid.extrude((pl * poly).faces()[0], V(51 * ad))
    saddle = (cyl(u0, ad, RP + 4, 51) & prism)
    parts.append(saddle)
    # plate on B
    nb = unit(FACE["normal"]); cb = np.array(FACE["origin"])
    zh = np.array([0, 0, 1.0]); eh = unit(np.cross(nb, zh))
    TB = GB
    plate = box(cb + TB / 2 * nb, zh, nb, 36, 36, TB)
    parts.append(plate)
    # beam
    S = cb + 3 * nb - 14.5 * zh
    E = A0 + 36 * m
    d = unit(E - S); L = np.linalg.norm(E - S)
    e = unit(np.cross(d, nb)); p = np.cross(d, e)
    pl = Plane(origin=V((S + E) / 2), x_dir=V(e), z_dir=V(d))
    beam = pl * Box(14, 8, L)
    beam = beam & box(cb + 30 * nb, zh, nb, 200, 200, 60)
    parts.append(beam)
    body = parts[0]
    for q in parts[1:]:
        body = body + q
    body = body - cyl(A0, ad, RP, 400)
    for k, b in BOLTS.items():
        a = unit(b["axis"]); P = np.array(b["position"])
        r = 2.75 if k[0] == "A" else 2.25
        body = body - cyl(P + 20 * a, a, r, 60)
    body = body.clean()
    return body


def envelopes(k, grip):
    b = BOLTS[k]; a = unit(b["axis"]); P = np.array(b["position"])
    hd, hh = b["head_diameter"], b["head_height"]
    sh = cyl(P + 10 * a, a, (2.5 if k[0] == "A" else 2.0), 20)
    hd_ = cyl(P + (grip + (hh + 1) / 2) * a, a, hd / 2 + 0.5, hh + 1)
    return sh + hd_


if __name__ == "__main__":
    body = build()
    export_step(body, str(HERE / "output.step"))
    print("wrote", HERE / "output.step", body.volume, body.is_valid, len(body.solids()))
    for k in BOLTS:
        g = GA if k[0] == "A" else GB
        iv = (body & envelopes(k, g))
        print(k, "envelope overlap vol", round(iv.volume, 3))
