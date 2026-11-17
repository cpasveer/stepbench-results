import json, math, traceback, Part, FreeCAD as App
V = App.Vector
D = "/home/user/stepbench-runs/tasks-freecad/facing-L3-s1/"
out = open(D + "scratch/check_local.txt", "w")
try:
    T = json.load(open(D + "task.json"))["poses"][0]["resolved"]
    part = Part.read(D + "output.step")
    fa = Part.read(D + "fixed_A.step"); fb = Part.read(D + "fixed_B.step")
    out.write("valid %s solids %d volume %.1f bbox %s\n" % (part.isValid(), len(part.Solids), part.Volume, part.BoundBox))
    for b in T["bolts"]:
        x, y, z = b["position"]; ax = b["axis"]; hd = b["head_diameter"]; hh = b["head_height"]
        sgn = ax[2]
        # head sits beyond the part: A bolts head above part (+z), B bolts head below (-z)
        if sgn > 0:
            head_base = -94.0 + 6.0
        else:
            head_base = -25.5 - 6.0
        env_r = hd / 2 + 0.5
        # head envelope: from part-surface to head top + 1 mm
        h0, h1 = sorted([head_base, head_base + sgn * (hh + 1.0)])
        env = Part.makeCylinder(env_r, h1 - h0, V(x, y, h0), V(0, 0, 1))
        inter = part.common(env).Volume
        # tool cylinder: head diameter, 30 mm beyond head top
        t0 = head_base + sgn * hh
        tz0, tz1 = sorted([t0, t0 + sgn * 30.0])
        tool = Part.makeCylinder(hd / 2, tz1 - tz0, V(x, y, tz0), V(0, 0, 1))
        tool_hit = part.common(tool).Volume + fa.common(tool).Volume + fb.common(tool).Volume
        # key sweep: outward sector (+-30 deg) radius 70 from bolt, heights head base .. head top+25
        kz0, kz1 = sorted([head_base - 5, head_base + sgn * (hh + 25.0)])
        out.write("%s envelope_overlap %.3f tool_hit %.3f\n" % (b["id"], inter, tool_hit))
    # overhang area in build direction +z: downward-facing faces steeper than 45 deg, not on bed
    zmin = part.BoundBox.ZMin
    oh = 0.0; n = 0
    for f in part.Faces:
        c = f.CenterOfMass
        nrm = f.normalAt(0.5, 0.5) if True else None
        if f.Surface.TypeId == "Part::GeomPlane":
            nn = f.normalAt(0, 0)
        else:
            continue
        if nn.z < -0.7071 and c.z > zmin + 0.01:
            oh += f.Area; n += 1
            out.write("  overhang plane area %.1f z %.2f normal %s\n" % (f.Area, c.z, nn))
    out.write("downward overhang area (planar, z-build) %.1f over %d faces\n" % (oh, n))
    # cylindrical faces with downward normal component
    cy = 0.0
    for f in part.Faces:
        if f.Surface.TypeId == "Part::GeomCylinder":
            # sample normal at mid-point
            u0, u1, v0, v1 = f.ParameterRange
            nn = f.normalAt((u0 + u1) / 2, (v0 + v1) / 2)
            if nn.z < -0.7071 and f.CenterOfMass.z > zmin + 0.01:
                cy += f.Area
    out.write("downward cylindrical overhang area %.1f\n" % cy)
except Exception:
    out.write(traceback.format_exc())
out.close()
