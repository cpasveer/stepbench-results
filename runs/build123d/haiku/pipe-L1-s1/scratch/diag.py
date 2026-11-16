import sys
sys.path.insert(0, "/home/user/stepbench-runs/tasks/pipe-L1-s1")
from build123d import import_step, Vector
import submission as S
p = import_step("/home/user/stepbench-runs/tasks/pipe-L1-s1/output.step")
PA = S.PA
for f in p.faces():
    c = f.center()
    if f.area < 3:
        v = c - PA.origin
        x = v.dot(PA.x_dir); y = v.dot(PA.y_dir); z = v.dot(PA.z_dir)
        rad = (y*y+z*z)**0.5
        print(f"area {f.area:.3f} centre pipe-local x={x:.2f} y={y:.2f} z={z:.2f} radial={rad:.2f} normal={f.normal_at(c).to_tuple() if hasattr(f.normal_at(c),'to_tuple') else (round(f.normal_at(c).X,2),round(f.normal_at(c).Y,2),round(f.normal_at(c).Z,2))}")
