import sys, math
sys.path.insert(0,'.')
import submission as S
from build123d import *
for HT in (20.0, 19.0, 18.0):
    S.HT=HT
    for pn1 in (24,30,34,38):
        for pt2 in (20,26,32):
            S.PN1,S.PT2=pn1,pt2
            ax=S.beam_axis()
            def ang(a,b): return math.degrees(math.atan2(b[1]-a[1],b[0]-a[0]))
            a1=ang(ax[0],ax[1]); a2=ang(ax[1],ax[2]); a3=ang(ax[2],ax[3])
            body=S.tube(ax)+S.keel(ax)
            c=S.tool_conflicts(body,extra=0.6)
            print(HT,pn1,pt2,'kinks',round(abs(a2-a1),1),round(abs(a3-a2),1),'conf',round(c,2),body.is_valid,flush=True)
