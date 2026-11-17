import traceback
out = open("/home/user/stepbench-runs/tasks-freecad/facing-L3-s1/scratch/fem_probe.txt", "w")
try:
    import Fem
    out.write("Fem ok\n")
except Exception as e:
    out.write("Fem import failed: %r\n" % (e,))
import shutil, os
out.write("ccx on PATH: %s\n" % shutil.which("ccx"))
out.write("gmsh on PATH: %s\n" % shutil.which("gmsh"))
out.close()
