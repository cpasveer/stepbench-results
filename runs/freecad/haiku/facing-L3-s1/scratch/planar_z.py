import traceback, Part, FreeCAD as App
D = "/home/user/stepbench-runs/tasks-freecad/facing-L3-s1/"
out = open(D + "scratch/planar_z.txt", "w")
try:
    s = Part.read(D + "output.step").Solids[0]
    for f in s.Faces:
        if f.Surface.TypeId == "Part::GeomPlane":
            c = f.CenterOfMass
            n = f.normalAt(0, 0)
            if abs(c.z + 30.5) < 0.6 or abs(c.z + 28.5) < 0.6:
                out.write("plane z %.2f normal %s area %.1f center %s\n" % (c.z, [round(x,2) for x in (n.x,n.y,n.z)], f.Area, [round(x,1) for x in (c.x,c.y)]))
except Exception:
    out.write(traceback.format_exc())
out.close()
