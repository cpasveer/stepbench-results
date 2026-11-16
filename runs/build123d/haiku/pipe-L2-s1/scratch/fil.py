import sys, pathlib
sys.path.insert(0, str(pathlib.Path(__file__).parent.parent))
from build123d import Vector
import submission as S
src = open(str(S.HERE/"submission.py")).read()
# rebuild up to the fillet point to test radii
import types
part = None
src2 = src.split("    # round off the two web junctions")[0] + "    return body, Qe, Pe\n"
src2 = src2.replace("def build():", "def build_pre():")
ns = {}
exec(compile(src2.replace('if __name__ == "__main__":','if False:'), "x", "exec"), ns)
body, Qe, Pe = ns["build_pre"]()
print("edges", len(body.edges()))
for rr in [1.5, 1.0, 0.8, 0.5]:
    joints = [e for e in body.edges() if min((e.center()-Qe).length, (e.center()-Pe).length) < 13.0]
    try:
        b2 = body.fillet(rr, joints); print("radius", rr, "ok with", len(joints), "edges")
        break
    except Exception as ex:
        print("radius", rr, "fail:", str(ex)[:60])
