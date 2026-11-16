import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).parent.parent))
from build123d import *
part = import_step(str(pathlib.Path(__file__).parent.parent/"output.step"))
V = part.volume; shell = 0
for f in part.faces():
    try: nz = abs(f.normal_at(f.center()).Z)
    except Exception: nz = 0
    shell += f.area * (0.8 if nz > 0.99 else 1.2)
shell = min(shell, V)
print("CAD cm3 %.2f  shell %.2f  est printed %.2f" % (V/1000, shell/1000, (shell + 0.2*(V-shell))/1000))
