import pathlib
from build123d import import_step, GeomType
p = import_step("/home/user/stepbench-runs/tasks/facing-L3-s1/output.step")
for f in p.faces():
    try:
        n = f.normal_at(f.center())
    except Exception:
        continue
    if n.Z < -0.05 and f.area > 1:
        c = f.center()
        print(f"nz={n.Z:+.2f} area={f.area:8.1f} center=({c.X:.1f},{c.Y:.1f},{c.Z:.1f}) type={f.geom_type}")
