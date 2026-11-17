#! python3
import os, traceback
HERE = "."
src = open(os.path.join(HERE, "submission.py")).read()
src = src.split("\ndef main():")[0]
g = {"__file__": os.path.join(HERE, "submission.py")}
out = []
try:
    exec(compile(src, "sub", "exec"), g)
    import Rhino.Geometry as rg
    body = g["build"]()[0]
    P = rg.Point3d(142.295, -36.324, -58.0)
    out.append("faces %d" % body.Faces.Count)
    for i, f in enumerate(body.Faces):
        bb = f.GetBoundingBox(True)
        if bb.Min.DistanceTo(P) < 1e9:
            ok, u, v = f.ClosestPoint(P)
            d = f.PointAt(u, v).DistanceTo(P)
            if d < 1.0:
                out.append("face %d dist %.3f area %.2f type %s" % (i, d, rg.AreaMassProperties.Compute(f.DuplicateFace(False)).Area, f.UnderlyingSurface().GetType().Name))
    for i, e in enumerate(body.Edges):
        if e.PointAtStart.DistanceTo(P) < 3.0 or e.PointAtEnd.DistanceTo(P) < 3.0:
            out.append("edge %d len %.3f %s -> %s" % (i, e.GetLength(), e.PointAtStart, e.PointAtEnd))
except Exception:
    out.append(traceback.format_exc())
open(os.path.join(HERE, "scratch", "dbg_out.txt"), "w").write("\n".join(out))
