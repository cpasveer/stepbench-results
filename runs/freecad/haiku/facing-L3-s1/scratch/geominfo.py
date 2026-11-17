import traceback, Part
out = open("/home/user/stepbench-runs/tasks-freecad/facing-L3-s1/scratch/inspect.txt","w")
try:
    for n in ["fixed_A.step","fixed_B.step"]:
        s = Part.read("/home/user/stepbench-runs/tasks-freecad/facing-L3-s1/"+n)
        out.write("%s solids %d bbox %s vol %.1f\n" % (n, len(s.Solids), s.BoundBox, s.Volume))
        for f in s.Faces:
            if f.Surface.TypeId == "Part::GeomPlane":
                out.write("  plane area %.1f normal %s center %s\n" % (f.Area, f.normalAt(0,0), f.CenterOfMass))
            else:
                out.write("  %s area %.1f center %s\n" % (f.Surface.TypeId, f.Area, f.CenterOfMass))
except Exception:
    out.write(traceback.format_exc())
out.close()
