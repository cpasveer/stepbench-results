import json, pathlib, numpy as np
TASK = json.loads(pathlib.Path("/home/user/stepbench-runs/tasks/pipe-L3-s1/task.json").read_text())
POSE = TASK["poses"][0]["resolved"]
BOLTS = {b["id"]: b for b in POSE["bolts"]}
FACE = POSE["mounting_faces"][0]
O_B = np.array(FACE["origin"]); N_B = np.array(FACE["normal"]); N_B /= np.linalg.norm(N_B)
R_B = np.array(TASK["poses"][0]["placements"]["B"]["rotation"])  # rows
EX_B = R_B[:,0]; EY_B = R_B[:,1]; EZ_B = R_B[:,2]   # columns of rotation -> rows given; use R as matrix: local->world = R @ local ?
# pipe frame
RA = np.array(TASK["poses"][0]["placements"]["A"]["rotation"])
O_A = np.array(TASK["poses"][0]["placements"]["A"]["origin"])
U_A = RA[:,0]  # pipe axis (local x)
R_PIPE = 34.0
def to_B(p):
    """world -> B local (x,y,z=normal), origin at face centre"""
    d = np.asarray(p) - O_B
    return np.array([d @ EX_B, d @ EY_B, d @ N_B])
def from_B(q):
    q = np.asarray(q)
    return O_B + q[0]*EX_B + q[1]*EY_B + q[2]*N_B
def to_A(p):
    d = np.asarray(p) - O_A
    return np.array([d @ RA[:,0], d @ RA[:,1], d @ RA[:,2]])
def from_A(q):
    return O_A + q[0]*RA[:,0] + q[1]*RA[:,1] + q[2]*RA[:,2]
