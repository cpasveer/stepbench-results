#! python3
import Rhino, Rhino.Geometry as rg, os
out=[]
def box(x0,x1,y0,y1,z0,z1):
    return rg.Box(rg.Plane.WorldXY, rg.Interval(x0,x1), rg.Interval(y0,y1), rg.Interval(z0,z1)).ToBrep()
a=box(0,10,0,10,0,10); b=box(5,20,2,8,0,10)
u=rg.Brep.CreateBooleanUnion([a,b],0.001)[0]
out.append("faces %d"%u.Faces.Count)
out.append("merge %s"%u.MergeCoplanarFaces(0.001))
out.append("faces %d"%u.Faces.Count)
# extrusion-based
def prism(pts,h):
    pp=[rg.Point3d(x,y,0) for x,y in pts]; pp.append(pp[0])
    c=rg.PolylineCurve(pp)
    ex=rg.Extrusion.Create(c,h,True)
    b=ex.ToBrep(True)
    out.append("prism faces %d solid %s"%(b.Faces.Count,b.IsSolid))
    return b
a=prism([(0,0),(10,0),(10,10),(0,10)],10); b=prism([(5,2),(20,2),(20,8),(5,8)],10)
u=rg.Brep.CreateBooleanUnion([a,b],0.001)[0]
out.append("ext faces %d"%u.Faces.Count)
out.append("merge %s"%u.MergeCoplanarFaces(0.001))
out.append("faces %d"%u.Faces.Count)
open(os.path.join(os.path.dirname(os.path.abspath(__file__.replace("file://",""))),"t.log"),"w").write("\n".join(out))
