import math, itertools
import FreeCAD as App, Part
SRC = open("/home/user/stepbench-runs/tasks-freecad/facing-L2-s1/submission.py").read().split("OUT = os.path.join")[0]
def run(tab_w, lift, k_cone, k_tab):
    src = SRC.replace("TAB_W = 4.0", "TAB_W = %r" % tab_w)
    src = src.replace("k_cone = 1.2 ", "k_cone = %r " % k_cone)
    src = src.replace("k_tab = 1.0 ", "k_tab = %r " % k_tab)
    src = src.replace("lift = 0.3 ", "lift = %r " % lift)
    g = {"__file__": "/home/user/stepbench-runs/tasks-freecad/facing-L2-s1/submission.py"}
    exec(compile(src, "sub", "exec"), g)
    s = g["build"]()
    s = s.Solids[0].removeSplitter() if len(s.Solids) == 1 else s.removeSplitter()
    s.exportStep("/home/user/stepbench-runs/tasks-freecad/facing-L2-s1/scratch/sw.step")
    r = Part.read("/home/user/stepbench-runs/tasks-freecad/facing-L2-s1/scratch/sw.step")
    minedge = min(e.Length for e in r.Edges)
    minface = min(f.Area for f in r.Faces)
    over = 0
    for f in r.Faces:
        if abs(f.BoundBox.ZMin + 94) < 1e-3 and abs(f.BoundBox.ZMax + 94) < 1e-3:
            continue
        u0,u1,v0,v1 = f.ParameterRange
        n = f.normalAt((u0+u1)/2, (v0+v1)/2)
        if f.Orientation == "Reversed": n = n * -1
        if n.z < -math.cos(math.radians(45)) - 1e-9: over += 1
    print("RESULT tab_w=%s lift=%s kc=%s kt=%s mem_valid=%s rt_valid=%s solids=%d minedge=%.3f minface=%.3f overhang_faces=%d faces=%d" % (
        tab_w, lift, k_cone, k_tab, s.isValid(), r.isValid(), len(r.Solids), minedge, minface, over, len(r.Faces)))
for tw, lf, kc in [(4.0,0.3,1.2),(3.0,0.3,1.2),(5.0,0.3,1.2),(4.0,0.3,1.5)]:
    try:
        run(tw, lf, kc, 1.0)
    except Exception as ex:
        print("RESULT error", tw, lf, kc, ex)
