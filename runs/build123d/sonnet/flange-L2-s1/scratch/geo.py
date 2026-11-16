import numpy as np
nA=np.array([.7384553406258838,.6743023875837234]);nB=np.array([-.4786918579404784,-.8779829754276512])
A=np.array([95.0,-91.0]);B=np.array([132.06606660467747,-30.747558502102073])
uA=np.array([-nA[1],nA[0]]);uB=np.array([-nB[1],nB[0]])
def fA(p): d=np.array(p)-A; return d@uA,d@nA
def fB(p): d=np.array(p)-B; return d@uB,d@nB
# wall corners in A frame
for u in(41,48):
  # intersect with s_B=4 : (A+u uA+s nA - B)@nB =4
  # s: ((A-B)+u uA)@nB + s nA@nB =4
  s=(4-(((A-B)+u*uA)@nB))/(nA@nB); p=A+u*uA+s*nA
  print(u,s,fB(p))
print(fB(A+41*uA+0*nA))
