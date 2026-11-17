import numpy as np
n=np.array([.3885173161164798,.9214414224885092,0]);ea=np.array([0,0,1.]);eb=np.cross(n,ea)
print('eb',eb, 'check', np.cross(ea,eb))
Bc=np.array([19.549595153015062,49.68566626427389,52.2472790015625])
ax=np.array([.25,.59,.77]);ax/=np.linalg.norm(ax);c=np.array([21,136.5,-7.])
t=np.array([0.20274448,-0.80715963,0.55442584]);s=np.cross(ax,t)
def W(x,y,z): return c+x*ax+y*t+z*s
P1=Bc+40*n
print('P1 local',[(P1-c)@v for v in (ax,t,s)])
for th in (-10,0,10,20):
  for x in (17.5,):
    r=34;N=W(x,r*np.cos(np.radians(th)),r*np.sin(np.radians(th)))
    d=N-P1;L=np.linalg.norm(d);d/=L
    print(th,x,L.round(1),'d.n',(d@n).round(2),'d.ea',(d@ea).round(2),'d.eb',(d@eb).round(2),'angle',np.degrees(np.arccos(d@n)).round(1))
