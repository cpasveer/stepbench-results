from ana import *
import math
a1=np.array([0,-0.573576436,0.819152044]);a2=np.array([0,0.573576436,0.819152044]);ex=np.array([1.,0,0])
# frame perpendicular to N: U,V
res=[]
for d in np.arange(0,14.1,1.0):
  for phi in range(0,360,10):
    dr=math.radians(d);ph=math.radians(phi)
    w=U*math.cos(ph)+V*math.sin(ph)
    u=N*math.cos(dr)+w*math.sin(dr)
    m=max(-(a2@u)*-1 if False else abs(a2@u),abs(a1@u),abs(ex@u))
    res.append((m,d,phi,u))
res.sort(key=lambda t:t[0])
for m,d,phi,u in res[:8]: print(round(m,3),d,phi,u.round(3),'a2',round(float(a2@u),3),'ex',round(float(ex@u),3),'a1',round(float(a1@u),3))
