import sys; sys.path.insert(0,'.')
from submission import *
import itertools
def samples(part):
    out=[]  # (normal, area_weight, face)
    for f in part.faces():
        try:
            us=np.linspace(0.05,0.95,6)
            bb=f.bounding_box()
            pts=[]
            if f.geom_type==GeomType.PLANE:
                n=f.normal_at(); out.append((np.array([n.X,n.Y,n.Z]),f.area,f)); continue
            # cylindrical/toroidal/other: sample via uv
            u0,u1,v0,v1=f._uv_bounds()
            for a in np.linspace(u0,u1,8):
                for b in np.linspace(v0,v1,8):
                    p=f.position_at(0,0) if False else None
                    try:
                        pt=f.position_at(a,b) if False else None
                    except Exception: pass
            # fallback: use surface normal at points
            for a in np.linspace(0.05,0.95,7):
                for b in np.linspace(0.05,0.95,7):
                    p=f.position_at(a,b)
                    n=f.normal_at(p); out.append((np.array([n.X,n.Y,n.Z]),f.area/49,f))
        except Exception as e:
            pass
    return out
def analyze(part,verbose=True):
    S=samples(part)
    # candidate ups: plane face normals reversed (bed on that face), plus axes
    cands=[]
    for f in part.faces():
        if f.geom_type==GeomType.PLANE and f.area>150:
            n=f.normal_at(); cands.append((-np.array([n.X,n.Y,n.Z]),f.area))
    best=None
    for up,area in cands:
        bad=0;worst=0
        for n,w,f in S:
            c=-(n@up)  # down component
            if c>0.7071+1e-6:
                if f.geom_type==GeomType.PLANE and abs(c-1)<1e-3: continue  # bed-parallel
                if f.geom_type==GeomType.CYLINDER and f.area<150: continue
                bad+=w; worst=max(worst,math.degrees(math.asin(min(1,c))))
        if verbose: print("up",np.round(up,2),"bed area",round(area),"overhang area",round(bad),"worst",round(worst,1))
        if best is None or bad<best[0]: best=(bad,up)
    return best
if __name__=="__main__":
    part=build(); print(analyze(part))
