"""Own approximate checks on a skeleton of rods (world coords). Not the official checker."""
import sys; sys.path.insert(0,'/home/user/stepbench-runs/tasks/pipe-L3-s1/scratch')
import numpy as np
from geo import *
from frame import solve
BIDS=['B1','B2','B3','B4']; AIDS=['A1','A2','A3','A4']
def bolt_info(k):
    b=BOLTS[k]; p=np.array(b['position']); a=np.array(b['axis']); a=a/np.linalg.norm(a)
    hd=b['head_diameter']; hh=b['head_height']; g=0
    return p,a,hd,hh
def sample_segments(segs, step=0.5):
    pts=[]
    for P,Q,r in segs:
        P=np.asarray(P,float);Q=np.asarray(Q,float); L=np.linalg.norm(Q-P)
        n=max(2,int(L/step))
        for t in np.linspace(0,1,n):
            pts.append((P+(Q-P)*t, r))
    return pts
def sector_report(segs, pad_pts, nsteps=72, hsel=None, tool_margin=0.5, rmax=75.0, verbose=False):
    """segs: list of (P,Q,r) world. pad_pts: list of (point, radius) for pads etc. returns dict."""
    pts=sample_segments(segs)+[(np.asarray(p),r) for p,r in pad_pts]
    P=np.array([p for p,_ in pts]); R=np.array([r for _,r in pts])
    out={}
    for k in BIDS+AIDS:
        p,a,hd,hh=bolt_info(k)
        # head top = seat + head height + grip (grip from seat to pad top). we use seat-relative heights
        # for B bolts the seat is the B face plane (grip g_B), for A bolts seat is pipe surface.
        # heights measured from head top (given by hh+gr)
        gr=GRIP[k]
        top=gr+hh
        rel=P-p; h=rel@a; rho_vec=rel-np.outer(h,a); rho=np.linalg.norm(rho_vec,axis=1)
        # TOOL access: cylinder radius hd/2+margin, from head top to head top+30
        tool=(h>top-0.01)&(h<top+30)&(rho<hd/2+tool_margin+R)
        tool_ok=not tool.any()
        # swept region heights
        band=(h>top+8)&(h<top+30)&(rho<rmax)
        ang=np.degrees(np.arctan2(rho_vec@AXES[k][1], rho_vec@AXES[k][0]))%360
        free_sectors=[]
        blocked=np.zeros(360,bool)
        for th,ok in zip(ang[band],[1]*band.sum()):
            pass
        hits=np.zeros(360,bool)
        idx=np.where(band)[0]
        for i_ in idx:
            lo=int(np.floor(ang[i_]-np.degrees(np.arcsin(min(1,(R[i_]+2.0)/max(rho[i_],1e-6))))) if rho[i_]>R[i_]+2 else 0)
            hi=int(np.ceil(ang[i_]+np.degrees(np.arcsin(min(1,(R[i_]+2.0)/max(rho[i_],1e-6))))) if rho[i_]>R[i_]+2 else 359)
            for d_ in range(lo,hi+1):
                hits[d_%360]=True
        # best free 70-degree window
        best=None
        for s in range(360):
            idxs=[(s+j)%360 for j in range(70)]
            if not hits[idxs].any(): best=s; break
        out[k]=dict(tool_ok=tool_ok, sector_start=best, blocked_deg=int(hits.sum()))
    return out
AXES={}
GRIP={'A1':6,'A2':6,'A3':6,'A4':6,'B1':4,'B2':4,'B3':4,'B4':4}
def init_axes():
    for k in BIDS+AIDS:
        p,a,hd,hh=bolt_info(k)
        e1=np.cross(a,[1,0,0]); 
        if np.linalg.norm(e1)<1e-3: e1=np.cross(a,[0,1,0])
        e1/=np.linalg.norm(e1); e2=np.cross(a,e1)
        AXES[k]=(e1,e2)
init_axes()
