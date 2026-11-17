from wall import *
res=[]
for u in (-13,-11,-9):
  for v in (0,3,5):
    for x1 in (-7.5,-10,-12):
      for Q0 in ((0,30),(-4,30)):
        E=bp(u,v,0);Q1=(E[1],E[2])
        wc=wallcheck(Q0,Q1,13,-23.5,x1)
        m=min(wc.values()) if wc else 9
        res.append((round(float(m),2),u,v,x1,Q0,{k:round(float(vv),1) for k,vv in wc.items() if vv<3}))
res.sort(key=lambda r:-r[0])
for r in res[:12]: print(r)
