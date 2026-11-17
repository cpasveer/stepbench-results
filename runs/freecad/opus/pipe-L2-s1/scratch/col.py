import numpy as np, json
exec(open('scratch/geo.py').read().split("print('B center")[0])
bu=RB[:,0];bv=RB[:,1]; Z=np.array([0,0,1.])
rad=lambda p: np.sin(np.radians(p))*ey+np.cos(np.radians(p))*ez
for tgt in [0,-8]:  # plate target u (below center)
 for xs in [-8,-4,0,4]:
  for ph in [10,15,20,25]:
    S=OA+xs*ex+41*rad(ph); PB=OB+6*n+tgt*bu
    c=PB-S; L=np.linalg.norm(c); c/=L
    print(tgt,xs,ph,'fromvert',round(np.degrees(np.arccos(c@Z)),1),'vs n',round(np.degrees(np.arccos(-c@n)),1),'vs rad',round(np.degrees(np.arccos(c@rad(ph))),1),'v',round(c@bv,2),'L',round(L))
