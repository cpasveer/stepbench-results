import sys; sys.path.insert(0,'.')
from submission import *
print("U.AX",U@AX,"N.AX",N@AX,"V.AX",V@AX,"UP.N",UP@N,"UP.AX",UP@AX)
print("saddle centre bed xy", bed_xy(P0+R_PIPE*LZ-15*AX), "h", (P0+R_PIPE*LZ-15*AX-E0)@UP)
for k,b in BOLTS.items():
    p=np.array(b["position"]); print(k, bed_xy(p), round(float((p-E0)@UP),2), "ax.up", round(float(np.array(b["axis"])@UP),3))
p=build()
for s in p.solids(): print(s.volume, s.bounding_box())
