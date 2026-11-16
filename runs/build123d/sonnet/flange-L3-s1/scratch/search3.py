import sys, math
sys.path.insert(0,'.')
import submission as S
for HT in (20.0,21.0):
    S.HT=HT
    for pn,pu in ((44,6),(40,5),(44,4),(46,7)):
        S.PN,S.PU=pn,pu
        ax=S.beam_axis()
        def ang(a,b): return math.degrees(math.atan2(b[1]-a[1],b[0]-a[0]))
        a=[ang(ax[i],ax[i+1]) for i in range(len(ax)-1)]
        body=S.tube(ax)+S.keel(ax)
        c=S.tool_conflicts(body,extra=0.4)
        print(HT,pn,pu,'kinks',[round(abs(a[i+1]-a[i]),1) for i in range(len(a)-1)],'conf',round(c,2),body.is_valid,len(body.solids()),flush=True)
