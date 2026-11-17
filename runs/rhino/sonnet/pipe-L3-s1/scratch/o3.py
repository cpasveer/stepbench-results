from ana import *
import math
a1=np.array([0,-0.573576436,0.819152044]);a2=np.array([0,0.573576436,0.819152044]);ex=np.array([1.,0,0])
def al(u,v): return math.degrees(math.acos(min(1,abs(u@v))))
# local heights of key points
pts={'plate c':c,'plate B1':bp(-12.5,-12.5,0),'P1':38*a1,'P2':38*a2,'P2xp':38*a2+23.5*ex,'P2xm':38*a2-23.5*ex}
cand=[np.array(x)/np.linalg.norm(x) for x in [(1,0,0),(-1,0,0),(0,1,0),(0,-1,0),(0,0,1),(0,0,-1),(.75,.42,-.52),(-.75,-.42,.52),(.7,0,.7),(-.7,0,.7)]]
for u in cand:
    print(u.round(2),'angles N,a1,a2:',[round(al(u,v)) for v in (N,a1,a2)],{k:round(float(p@u),1) for k,p in pts.items()})
