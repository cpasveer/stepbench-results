import sys, math, itertools
sys.path.insert(0,'.')
import submission as S
from build123d import *
oA,nA,tA=S.frame("A"); oB,nB,tB=S.frame("B")
S.HT=float(sys.argv[1]) if len(sys.argv)>1 else 21.0
res=[]
for pn1 in (12,16,20,24,28):
    for pt2 in (24,30,36,42):
        S.PN1,S.PT2=pn1,pt2
        ax=S.beam_axis()
        # kink angles
        def ang(a,b): return math.degrees(math.atan2(b[1]-a[1],b[0]-a[0]))
        a1=ang(ax[0],ax[1]); a2=ang(ax[1],ax[2]); a3=ang(ax[2],ax[3])
        k1=abs(a2-a1); k2=abs(a3-a2)
        try:
            body=S.tube(ax)+S.keel(ax)
            c=S.tool_conflicts(body,extra=0.6)
            ok=body.is_valid and len(body.solids())==1
        except Exception as e:
            c=-1; ok=False
        print(pn1,pt2,'kinks',round(k1,1),round(k2,1),'conf',round(c,2),ok,flush=True)
