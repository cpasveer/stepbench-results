import numpy as np
n=np.array([0.64037208,-0.38434937,-0.66498057]); xB=np.array([0.768,0.3205,0.5544]); yB=np.array([0,-0.8658,0.5004])
def rad(t): return np.array([0,-np.sin(t),np.cos(t)])  # positive t toward -y? A1 at y=-19.5 => t=+35
N={'Bseat':-n,'Bhead':n,'+xB':xB,'-xB':-xB,'+yB':yB,'-yB':-yB}
for t in np.radians(np.arange(-45,46,5)): N['sad%d'%round(np.degrees(t))]=-rad(t)
for t in np.radians([35,-35]): N['head%d'%round(np.degrees(t))]=rad(t)
best=[]
for th in np.radians(np.arange(0,181,2)):
  for ph in np.radians(np.arange(0,360,2)):
    d=np.array([np.sin(th)*np.cos(ph),np.sin(th)*np.sin(ph),np.cos(th)])
    m={k:v@d for k,v in N.items()}
    worst=min(m.values()); best.append((worst,tuple(d.round(3)),min(m,key=m.get)))
best.sort(reverse=True)
for b in best[:5]: print(b)
d=np.array([1,0,0.]);print({k:round(v@d,3) for k,v in N.items() if v@d<-0.5})
