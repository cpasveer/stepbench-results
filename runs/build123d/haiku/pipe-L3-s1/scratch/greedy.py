import sys, time; sys.path.insert(0,'/home/user/stepbench-runs/tasks/pipe-L3-s1/scratch')
import numpy as np
from pin import pin_k
from p4 import Anodes, Bnodes, from_B, to_B
from geo import O_A, U_A
BL=np.array([[-12.5,-12.5],[12.5,-12.5],[12.5,12.5],[-12.5,12.5]])
def bolt_dist_ok(q, r):
    """q: B-local point. True if outside B-bolt keep-out cylinders (radius 4.8+r) for z in 3..40"""
    for bx,by in BL:
        if q[2]>3 and q[2]<42 and np.hypot(q[0]-bx,q[1]-by)<4.8+r: return False
    return True
def seg_ok(P,Q,r):
    # sample along segment in B-local
    L=np.linalg.norm(Q-P); n=max(2,int(L/1.0))
    for t in np.linspace(0,1,n):
        q=P+(Q-P)*t
        if not bolt_dist_ok(q,r): return False
    return True
def pipe_ok(P,Q,r,endpoints_A=True):
    # distance from pipe axis (B-local coords of axis) must be >= 34 + r + 0.5 except near A nodes
    L=np.linalg.norm(Q-P); n=max(2,int(L/1.0))
    for t in np.linspace(0,1,n):
        q=P+(Q-P)*t
        d=q-PO; ax=d-(d@PU)*PU
        if np.linalg.norm(ax)<34+r+0.5 and (min(t,1-t)*L>7): return False
    return True
PO=None; PU=None
def setup_pipe():
    global PO,PU
    from p4 import to_B as tb
    PO=tb(O_A) ; PU=np.array([U_A@ __import__('geo').EX_B, U_A@__import__('geo').EY_B, U_A@__import__('geo').N_B])
    PU=PU/np.linalg.norm(PU)
setup_pipe()
