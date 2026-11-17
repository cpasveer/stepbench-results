import ana
from ana import *
newc=[]
for nm,p0,ax,r,ln in ana.cyl:
    if nm[0]=='B': p0=p0+ax*(9.0-3.5)
    newc.append((nm,p0,ax,r,ln))
ana.cyl[:]=newc
import wall as W
for ue in (-28,-34):
  for ve in (0,4,8):
    E=bp(ue,ve,0);Q1=(E[1],E[2])
    for w in (14,16,20):
      out=[]
      for xt in (-6,-3,0,3):
        wc=W.wallcheck((-4,30),Q1,w,-20.0,xt,n_min=0.4)
        out.append({k:round(float(v),1) for k,v in wc.items() if v<2 and k[0]=='B'})
      print('end',ue,ve,'w',w,out)
