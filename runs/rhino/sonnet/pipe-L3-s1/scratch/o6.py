from ana import *
import math
a1=np.array([0,-0.573576436,0.819152044]);a2=np.array([0,0.573576436,0.819152044]);ex=np.array([1.,0,0])
tg1=np.cross(a1,ex);tg2=np.cross(a2,ex)
vs=[a1,tg1,a2,tg2,N,V]
res=[]
for th in np.linspace(0,math.pi,181):
  for ph in np.linspace(0,2*math.pi,360,endpoint=False):
    u=np.array([math.sin(th)*math.cos(ph),math.sin(th)*math.sin(ph),math.cos(th)])
    m=max(abs(u@v) for v in vs)
    # pads at equal height at x ends? prefer |u.y|... record
    res.append((m,u))
res.sort(key=lambda t:t[0])
seen=[]
for m,u in res[:4000:400]:
    print(round(m,3),u.round(2),'U',round(float(u@U),2),'ex',round(float(u@ex),2),'N',round(float(u@N),2))
