import ana
from ana import *
newc=[]
for nm,p0,ax,r,ln in ana.cyl:
    if nm[0]=='B': p0=p0+ax*(9.0-3.5)
    newc.append((nm,p0,ax,r,ln))
ana.cyl[:]=newc
import wall as W
rows=[]
for ue in (-28,-34,-40,-46):
  E=bp(ue,0,0);Q1=(E[1],E[2])
  for w in (12,14,16):
    out=[]
    for xt in (-6,-3,0,3,6):
      wc=W.wallcheck((-4,30),Q1,w,-20.0,xt,n_min=0.4)
      m=min([v for k,v in wc.items() if k[0]=='B'] or [9])
      out.append(round(float(m),1))
    print('end',ue,'w',w,'tops -6,-3,0,3,6 clearance',out)
