from ana import *
import math
a1=np.array([0,-0.573576436,0.819152044]);a2=np.array([0,0.573576436,0.819152044]);ex=np.array([1.,0,0])
tg1=np.cross(a1,ex);tg2=np.cross(a2,ex)
vs={'U':U,'V':V,'N':N,'a1':a1,'tg1':tg1,'ex':ex,'a2':a2,'tg2':tg2}
res=[]
for th in np.linspace(0,math.pi,181):
  for ph in np.linspace(0,2*math.pi,360,endpoint=False):
    u=np.array([math.sin(th)*math.cos(ph),math.sin(th)*math.sin(ph),math.cos(th)])
    m=max(abs(u@v) for v in vs.values())
    res.append((m,u))
res.sort(key=lambda t:t[0])
for m,u in res[:6]:
    print(round(m,3),u.round(2),{k:round(float(u@v),2) for k,v in vs.items()})
