import json, sys
import numpy as np
from frame import solve
T = json.load(open("/home/user/stepbench-runs/tasks-freecad/flange-L3-s1/task.json"))
R = T["poses"][0]["resolved"]
OA = np.array(R["mounting_faces"][0]["origin"]); nA = np.array(R["mounting_faces"][0]["normal"])
OB = np.array(R["mounting_faces"][1]["origin"]); nB = np.array(R["mounting_faces"][1]["normal"])
Z = np.array([0,0,1.0]); tA = np.cross(Z, nA); uB = np.cross(Z, nB)
def FA(a,b,z): return OA + nA*a + tA*b + Z*z
def onB(b,z):
    a = np.dot(OB-OA-tA*b-Z*z, nB)/np.dot(nA,nB)
    return FA(a,b,z)
def loc(p):
    v = np.asarray(p,float)-OA; return np.array([v@nA, v@tA, v[2]])
Ahole = [np.array(b["position"],float) for b in R["bolts"] if b["id"][0]=="A"]
Bhole = [np.array(b["position"],float) for b in R["bolts"] if b["id"][0]=="B"]

def rect_I(w, h):
    return w*h**3/12.0, h*w**3/12.0

def spoke_props(w, t):
    Iw, Ip = rect_I(w, t)
    I = min(Iw, Ip)
    return (w*t, I, I, 2*I)

def K_of(E, nodes_world, elems, fixed_prefix="Afix", load="Bc", rigid_prefix="Bbolt"):
    """elems: list of (n1, n2, (A,Iy,Iz,J)). Returns 6x6 compliance at load node (local coords)."""
    G = E/(2*(1+0.35))
    nodes = {k: loc(v) for k,v in nodes_world.items()}
    els = [(a,b,(E,G)+tuple(p)) for a,b,p in elems]
    STIFF = 1e4*E
    for k in nodes:
        if k.startswith(rigid_prefix):
            els.append((load, k, (STIFF, STIFF/2.6, 1e4*STIFF/E, 1e5*STIFF/E, 1e5*STIFF/E, 1e5*STIFF/E)))
    fixed = [k for k in nodes if k.startswith(fixed_prefix)]
    return solve(nodes, els, fixed, load, [])
