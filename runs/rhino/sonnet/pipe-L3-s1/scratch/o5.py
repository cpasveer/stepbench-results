from ana import *
import math
a1=np.array([0,-0.573576436,0.819152044]);a2=np.array([0,0.573576436,0.819152044]);ex=np.array([1.,0,0])
def al(u,v): return math.degrees(math.acos(min(1,abs(u@v))))
res=[]
for th in np.linspace(0,math.pi,181):
  for ph in np.linspace(0,2*math.pi,360,endpoint=False):
    u=np.array([math.sin(th)*math.cos(ph),math.sin(th)*math.sin(ph),math.cos(th)])
    res.append((min(al(u,N),al(u,a1),al(u,a2)),u))
res.sort(key=lambda t:-t[0])
for m,u in res[:4]: print(round(m,1),u.round(2),[round(al(u,v)) for v in (N,a1,a2,ex,U,V)])
