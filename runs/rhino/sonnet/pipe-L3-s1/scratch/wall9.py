import ana
from ana import *
newc=[]
for nm,p0,ax,r,ln in ana.cyl:
    if nm[0]=='B': p0=p0+ax*(9.0-3.5)
    newc.append((nm,p0,ax,r,ln))
ana.cyl[:]=newc
import wall as W
for ue in (-28,-32,-36,-40):
  E=bp(ue,0,0);Q1=(E[1],E[2])
  for w in (16,):
    for xt in (-6,-3,0):
      wc=W.wallcheck((-4,30),Q1,w,-20.0,xt,n_min=0.4)
      print(ue,w,xt,{k:round(float(v),1) for k,v in wc.items() if v<4})
