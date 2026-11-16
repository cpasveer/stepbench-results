import json, math, sys
sys.path.insert(0, ".")
from build123d import import_step, Vector
import submission as S
part = import_step("output.step")
print("valid", part.is_valid, "solids", len(part.solids()), "vol", round(part.volume,1))
A = import_step("fixed_A.step"); B = import_step("fixed_B.step")
for name, f in (("A", A), ("B", B)):
    inter = part.intersect(f)
    v = inter.volume if inter is not None else 0.0
    print("overlap with fixed", name, round(v, 4))
    bb = f.bounding_box(); print(" fixed", name, "bbox", [round(c,1) for c in (bb.min.X,bb.min.Y,bb.min.Z)], [round(c,1) for c in (bb.max.X,bb.max.Y,bb.max.Z)])
bb = part.bounding_box(); print("part bbox", [round(c,1) for c in (bb.min.X,bb.min.Y,bb.min.Z)], [round(c,1) for c in (bb.max.X,bb.max.Y,bb.max.Z)])
# bolt checks: every bolt axis must pass through part (hole) and the seat plane contact
for b in S.POSE["bolts"]:
    p = Vector(*b["position"]); ax = Vector(*b["axis"]).normalized()
    # sample along axis from fixed surface outward
    inside = []
    for t in [0.5, 2.0, 3.0, 4.0, 4.5, 5.0, 5.5, 6.0]:
        q = p + ax * t
        inside.append(part.is_inside(q) if hasattr(part,'is_inside') else None)
    # hole test: point at radius 0.6*hole dia off-axis should be outside (in the hole)
    print(b["id"], "axis-samples inside(0.5..6):", inside)
