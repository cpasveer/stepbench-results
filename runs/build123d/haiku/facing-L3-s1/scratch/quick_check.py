import json, math, pathlib
from build123d import import_step, Pos, Cylinder, Align
HERE = pathlib.Path("/home/user/stepbench-runs/tasks/facing-L3-s1")
part = import_step(str(HERE / "output.step"))
solids = part.solids()
print("solids", len(solids), "valid", part.is_valid, "volume", round(part.volume, 1))
A = import_step(str(HERE / "fixed_A.step"))
B = import_step(str(HERE / "fixed_B.step"))
print("overlap A", round((part & A).volume if (part & A) else 0, 4), "overlap B", round((part & B).volume if (part & B) else 0, 4))
TASK = json.loads((HERE / "task.json").read_text())
bolts = TASK["poses"][0]["resolved"]["bolts"]
bb = part.bounding_box()
print("bbox", bb.min, bb.max)
# head keep-outs: A heads above z=-90 up to head top+1 ; B heads below z=-29.5
for b in bolts:
    x, y, z = b["position"]
    hr = b["head_diameter"] / 2 + 0.5
    hh = b["head_height"]
    if b["axis"][2] > 0:
        z0, z1 = z + 4.0 - 0.0, z + hh + 1.0  # placeholder; grip set below
        zs = (z + 4.0, z + 4.0 + hh + 1.0)
        zs = (z + 4.0, z + 4.0 + 30)  # tool cylinder above head (grip 4)
        keep_head = (z + 4.0, z + 4.0 + hh + 1.0)
    else:
        keep_head = (z - 4.0 - hh - 1.0, z - 4.0)
        zs = (z - 4.0 - hh - 30.0, z - 4.0 - hh)
    cyl = Pos(x, y, zs[0]) * Cylinder(hr, zs[1] - zs[0], align=(Align.CENTER, Align.CENTER, Align.MIN))
    tool = Pos(x, y, zs[0]) * Cylinder(b["head_diameter"] / 2, zs[1] - zs[0], align=(Align.CENTER, Align.CENTER, Align.MIN))
    print(b["id"], "head-keepout vol", round((part & cyl).volume if (part & cyl) else 0, 4), "tool-cyl vol", round((part & tool).volume if (part & tool) else 0, 4))
