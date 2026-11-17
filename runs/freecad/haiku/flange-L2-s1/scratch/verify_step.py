import Part
s = Part.read("/home/user/stepbench-runs/tasks-freecad/flange-L2-s1/output.step")
print("STEPCHK solids=%d valid=%s closed=%s volume=%.1f faces=%d" % (len(s.Solids), s.isValid(), all(sh.isClosed() for sh in s.Shells) if s.Shells else None, s.Volume, len(s.Faces)))
