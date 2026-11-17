from ana import *
import math
a1=np.array([0,-0.573576436,0.819152044]);a2=np.array([0,0.573576436,0.819152044])
D=np.array([-0.44,0.09,0.89]);D/=np.linalg.norm(D)
def ang_line(u,v): return math.degrees(math.acos(min(1,abs(u@v))))
best=[]
for th in np.linspace(0,math.pi,181):
  for ph in np.linspace(0,2*math.pi,360,endpoint=False):
    u=np.array([math.sin(th)*math.cos(ph),math.sin(th)*math.sin(ph),math.cos(th)])
    m=min(ang_line(u,N)-45,ang_line(u,a1)-45,ang_line(u,a2)-45,45-ang_line(u,D))
    best.append((m,u))
best.sort(key=lambda t:-t[0])
for m,u in best[:5]: print(round(m,1),u.round(2),[round(ang_line(u,v)) for v in (N,a1,a2,D)])
