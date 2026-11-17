"""Local stiffness check: mesh a STEP with gmsh (quadratic tets), solve with CalculiX.
Fixed: all nodes on z = -94 (A seat). Load: 8.83 N in +x spread over nodes on z = -25.5 (B seat).
Prints the stiffness F/delta (N/mm) using the mean x-displacement of the load nodes."""
import subprocess, sys, os, math
GMSH = "/opt/freecad/squashfs-root/usr/bin/gmsh"
CCX = "/opt/freecad/squashfs-root/usr/bin/ccx"
step, tag = sys.argv[1], sys.argv[2]
E = float(sys.argv[3]) if len(sys.argv) > 3 else 1000.0   # MPa placeholder, scale later
work = os.path.dirname(os.path.abspath(__file__)) + "/" + tag
os.makedirs(work, exist_ok=True)
msh = work + "/mesh.msh"
subprocess.run([GMSH, step, "-3", "-order", "2", "-clmax", os.environ.get("CLMAX", "2.5"), "-clmin", os.environ.get("CLMIN", "1.0"), "-format", "msh2", "-o", msh, "-v", "0"], check=True)
lines = open(msh).read().split("\n")
i = lines.index("$Nodes"); n = int(lines[i+1]); nodes = {}
for k in range(n):
    p = lines[i+2+k].split(); nodes[int(p[0])] = (float(p[1]), float(p[2]), float(p[3]))
nodes_of = lambda nid: nodes[nid]
j = lines.index("$Elements"); m = int(lines[j+1]); tets = []
for k in range(m):
    p = list(map(int, lines[j+2+k].split()))
    if p[1] == 11:   # 10-node tetra
        g = p[3 + p[2]:]            # gmsh order: corners 0-3, mids (01,12,02,03,23,13)
        c = [g[0], g[1], g[2], g[3]]
        mids = [g[4], g[5], g[6], g[7], g[9], g[8]]   # CalculiX order: (01,12,02,03,13,23)
        A_, B_, C_, D_ = [nodes[x] for x in c]
        det = (B_[0]-A_[0])*((C_[1]-A_[1])*(D_[2]-A_[2])-(C_[2]-A_[2])*(D_[1]-A_[1])) \
            - (B_[1]-A_[1])*((C_[0]-A_[0])*(D_[2]-A_[2])-(C_[2]-A_[2])*(D_[0]-A_[0])) \
            + (B_[2]-A_[2])*((C_[0]-A_[0])*(D_[1]-A_[1])-(C_[1]-A_[1])*(D_[0]-A_[0]))
        if det < 0:   # swap corners 0 and 1 and remap the midside nodes
            c = [c[1], c[0], c[2], c[3]]
            mids = [mids[0], mids[2], mids[1], mids[4], mids[3], mids[5]]
        tets.append(c + mids)
inp = work + "/job.inp"
zA = -94.0; zB = -25.5; tol = 0.05
fixed = [nid for nid, (x, y, z) in nodes.items() if abs(z - zA) < tol]
load = [nid for nid, (x, y, z) in nodes.items() if abs(z - zB) < tol]
if os.environ.get("LOADMODE", "all") in ("hole", "holering"):
    import json as _j
    _T = _j.load(open("/home/user/stepbench-runs/tasks-freecad/facing-L3-s1/task.json"))["poses"][0]["resolved"]
    _B = [(b["position"][0], b["position"][1]) for b in _T["bolts"] if b["axis"][2] < 0]
    hr = float(os.environ.get("HOLER", "2.75"))
    hole_nodes = [nid for nid, (x, y, z) in nodes.items() if nid in set(n for t in tets for n in t) and -30.6 <= z <= -25.4 and any(abs(math.hypot(x-bx, y-by) - hr) < 0.25 for bx, by in _B)]
    if os.environ.get("LOADMODE") == "hole":
        load = hole_nodes
    else:
        load = list(set(load) & set(nid for nid in load)) + hole_nodes
    load = sorted(set(load))
if os.environ.get("LOADMODE", "all") == "bolt":
    import json as _j
    _T = _j.load(open("/home/user/stepbench-runs/tasks-freecad/facing-L3-s1/task.json"))["poses"][0]["resolved"]
    _B = [(b["position"][0], b["position"][1]) for b in _T["bolts"] if b["axis"][2] < 0]
    load = [nid for nid in load if any(math.hypot(nodes[nid][0]-bx, nodes[nid][1]-by) <= float(os.environ.get("LOADR", "4.25")) for bx, by in _B)]
used = set(nid for t in tets for nid in t)
fixed = [f for f in fixed if f in used]; load = [l for l in load if l in used]
print("nodes", len(nodes), "tets", len(tets), "fixed", len(fixed), "load", len(load))
with open(inp, "w") as f:
    f.write("*NODE\n")
    for nid in sorted(used):
        x, y, z = nodes[nid]; f.write("%d, %.6f, %.6f, %.6f\n" % (nid, x, y, z))
    f.write("*ELEMENT, TYPE=C3D10, ELSET=EALL\n")
    for k, t in enumerate(tets):
        # gmsh and CalculiX share the C3D10 node order
        f.write("%d, %s\n" % (k+1, ", ".join(str(x) for x in t)))
    f.write("*MATERIAL, NAME=PLA\n*ELASTIC\n%.1f, 0.35\n" % E)
    f.write("*SOLID SECTION, ELSET=EALL, MATERIAL=PLA\n")
    f.write("*BOUNDARY\n")
    for nid in fixed:
        f.write("%d, 1, 3, 0.0\n" % nid)
    f.write("*STEP\n*STATIC\n*CLOAD\n")
    F = 8.83
    if os.environ.get("LOADKIND", "lateral") == "torsion":
        T = F * 68.95
        xc = sum(nodes[n][0] for n in load) / len(load)
        yc = sum(nodes[n][1] for n in load) / len(load)
        Sr2 = sum((nodes[n][0]-xc)**2 + (nodes[n][1]-yc)**2 for n in load)
        k = T / Sr2
        for nid in load:
            rx, ry = nodes[nid][0]-xc, nodes[nid][1]-yc
            f.write("%d, 1, %.6f\n%d, 2, %.6f\n" % (nid, -k*ry, nid, k*rx))
    elif os.environ.get("LOADKIND", "lateral") == "tilt":
        # couple about the y axis through the seat centre: F_z = c*(x - xc), sum F_z (x - xc) = M
        M = F * 68.95
        xc = sum(nodes[n][0] for n in load) / len(load)
        Sxx = sum((nodes[n][0] - xc) ** 2 for n in load)
        c = M / Sxx
        for nid in load:
            f.write("%d, 3, %.6f\n" % (nid, c * (nodes[nid][0] - xc)))
    else:
        for nid in load:
            f.write("%d, 1, %.6f\n" % (nid, F / len(load)))
    f.write("*NSET, NSET=LOADN\n")
    for nid in load:
        f.write("%d,\n" % nid)
    f.write("*NODE PRINT, NSET=LOADN\nU\n*END STEP\n")
subprocess.run([CCX, "-i", work + "/job"], cwd=work, check=True, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
# read .dat displacements for load nodes
dat = open(work + "/job.dat").read().split("\n")
disp = {}
for line in dat:
    p = line.split()
    if len(p) == 4:
        try:
            nid = int(p[0]); disp[nid] = (float(p[1]), float(p[2]), float(p[3]))
        except ValueError:
            pass
if os.environ.get("LOADKIND", "lateral") == "torsion":
    xc = sum(nodes[n][0] for n in load) / len(load)
    yc = sum(nodes[n][1] for n in load) / len(load)
    Sr2 = sum((nodes[n][0]-xc)**2 + (nodes[n][1]-yc)**2 for n in load)
    phi = sum(-disp[l][0]*(nodes[l][1]-yc) + disp[l][1]*(nodes[l][0]-xc) for l in load if l in disp) / Sr2
    T = F * 68.95
    K = T / phi
    print("tag", tag, "E", E, "torsion phi", phi, "K_tors Nmm/rad", round(K, 1), "equiv N/mm (K/L^2)", round(K/68.95**2, 4))
elif os.environ.get("LOADKIND", "lateral") == "tilt":
    xc = sum(nodes[n][0] for n in load) / len(load)
    Sxx = sum((nodes[n][0] - xc) ** 2 for n in load)
    th = sum(disp[l][2] * (nodes[l][0] - xc) for l in load if l in disp) / Sxx   # rad
    M = F * 68.95
    K = M / th
    print("tag", tag, "E", E, "tilt theta", th, "K_rot Nmm/rad", round(K, 1), "equiv N/mm (K/L^2)", round(K / 68.95**2, 4))
else:
    ux = [disp[l][0] for l in load if l in disp]
    d = sum(ux) / len(ux)
    print("tag", tag, "E", E, "mean load-node ux", round(d, 6), "stiffness N/mm", round(F / d, 4) if d else None)
