from wall import *
for u in (-25,-22):
  for v in (0,):
    E=bp(u,v,0);Q1=(E[1],E[2])
    for Q0 in ((-4,30),(0,30),(4,30)):
      for w in (12,14):
        for x1 in (-8,-12):
          wc=wallcheck(Q0,Q1,w,-23.5,x1)
          print(u,v,Q0,w,x1,{k:round(float(vv),1) for k,vv in wc.items() if vv<3.5})
