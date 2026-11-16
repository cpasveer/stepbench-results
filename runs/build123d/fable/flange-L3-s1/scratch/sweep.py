import sys, pathlib, subprocess, json
sys.path.insert(0, str(pathlib.Path(__file__).parent.parent))
from build123d import export_step
import submission as S
H = pathlib.Path(__file__).parent
variants = json.loads(sys.argv[1])
for name, ov in variants.items():
    for k, v in ov.items(): setattr(S, k, v)
    p = S.build(); f = H / f"var_{name}.step"; export_step(p, str(f))
    V = p.volume; shell = 0
    for fc in p.faces():
        nz = abs(fc.normal_at(fc.center()).Z); shell += fc.area * (0.8 if nz > 0.99 else 1.2)
    est = (shell + 0.2 * (V - shell)) / 1000
    out = subprocess.run([sys.executable, str(H / "fe.py"), str(f), sys.argv[2] if len(sys.argv) > 2 else "2.0", sys.argv[3] if len(sys.argv) > 3 else "seat"], capture_output=True, text=True).stdout
    tr = [l for l in out.splitlines() if "transl" in l][0].split("[")[1].split("]")[0]
    ro = [l for l in out.splitlines() if "equivalent" in l][0].split("= [")[1].split("]")[0]
    print(f"{name:10s} {ov} solids {len(p.solids())} V {V/1000:.1f} est {est:.1f}  trans [{tr}] rot [{ro}]", flush=True)
