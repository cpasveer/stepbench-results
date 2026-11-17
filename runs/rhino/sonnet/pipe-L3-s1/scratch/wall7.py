import ana
from ana import *
newc=[]
for nm,p0,ax,r,ln in ana.cyl:
    if nm[0]=='B': p0=p0+ax*(9.0-3.5)
    newc.append((nm,p0,ax,r,ln))
ana.cyl[:]=newc
import wall as W
E=bp(-28,0,0);Q1=(E[1],E[2])
for w in (12,16):
  for xt in (-6,-4,-2,0):
    wc=W.wallcheck((-4,30),Q1,w,-23.5,xt,n_min=0.4)
    print(w,xt,{k:round(float(v),1) for k,v in wc.items() if v<4})
