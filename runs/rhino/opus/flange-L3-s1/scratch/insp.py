#! python3
import Rhino, Rhino.Geometry as rg, os
HERE="."
out=[]
doc=Rhino.RhinoDoc.CreateHeadless(None)
try:
    doc.Import(os.path.join(HERE,"output.step"))
    for o in doc.Objects:
        b=o.Geometry
        if isinstance(b,rg.Extrusion): b=b.ToBrep()
        out.append("brep faces %d"%b.Faces.Count)
        for i,f in enumerate(b.Faces):
            ok,pl=f.TryGetPlane(0.01)
            am=rg.AreaMassProperties.Compute(f.DuplicateFace(False))
            c=am.Centroid
            s="%2d %-14s area %8.1f c(%.1f %.1f %.1f)"%(i,f.UnderlyingSurface().GetType().Name,am.Area,c.X,c.Y,c.Z)
            if ok: s+=" n(%.3f %.3f %.3f) d %.2f"%(pl.Normal.X,pl.Normal.Y,pl.Normal.Z, pl.Normal*rg.Vector3d(pl.Origin))
            out.append(s)
        vs=[v.Location for v in b.Vertices]
        for i in range(len(vs)):
            for j in range(i+1,len(vs)):
                d=vs[i].DistanceTo(vs[j])
                if 3.0<d<4.0: out.append("v %d %d %.3f (%.1f %.1f %.1f) (%.1f %.1f %.1f)"%(i,j,d,vs[i].X,vs[i].Y,vs[i].Z,vs[j].X,vs[j].Y,vs[j].Z))
finally:
    doc.Dispose()
open(os.path.join(HERE,"scratch/insp.log"),"w").write("\n".join(out))
open(os.path.join(HERE,"submission.log"),"w").write("ok: insp\n")
