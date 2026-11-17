import FreeCAD as App, Part
s=Part.read("/home/user/stepbench-runs/tasks-freecad/flange-L3-s1/output.step")
for i,so in enumerate(s.Solids):
    bb=so.BoundBox
    print("SOLID",i,"vol %.0f"%so.Volume, "bb (%.1f,%.1f,%.1f)-(%.1f,%.1f,%.1f)"%(bb.XMin,bb.YMin,bb.ZMin,bb.XMax,bb.YMax,bb.ZMax))
