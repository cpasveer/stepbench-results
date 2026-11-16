import sys; sys.path.insert(0,'.')
from submission import *
a1=np.array([0,.5736,.8192]);a3=np.array([0,-.5736,.8192])
S=np.array([12.,0,40]); P=BO+BN*3
vec=P-S
best=[]
for th in np.linspace(0,math.pi,181):
  for ph in np.linspace(0,2*math.pi,361):
    up=np.array([math.sin(th)*math.cos(ph),math.sin(th)*math.sin(ph),math.cos(th)])
    if max(abs(up@a1),abs(up@a3),abs(up@BN))<=0.68:
        best.append((abs(up@vec)/np.linalg.norm(vec),up))
best.sort(key=lambda t:-t[0])
for b in best[:5]: print(round(math.degrees(math.asin(b[0])),1),np.round(b[1],2))
print(np.linalg.norm(vec),vec)
