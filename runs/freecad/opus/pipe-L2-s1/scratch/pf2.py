import numpy as np, json,sys
exec(open('scratch/geo.py').read().split("print('B center")[0])
bu=RB[:,0];bv=RB[:,1]
rad=lambda p: np.sin(np.radians(p))*ey+np.cos(np.radians(p))*ez
d=np.array([float(v) for v in sys.argv[1:4]]);d/=np.linalg.norm(d)
U=-d; X=np.cross(U,ex);X/=np.linalg.norm(X);Y=np.cross(U,X)
P=lambda p: np.round([(np.array(p)-OA)@X,(np.array(p)-OA)@Y,(np.array(p)-OA)@U],1)
for b in T['poses'][0]['resolved']['bolts']:
    print(b['id'],P(b['position']),'axis',np.round([np.array(b['axis'])@v for v in (X,Y,U)],2))
for su in (-17,17):
  for sv in (-17,17): print('plate',su,sv,P(OB+su*bu+sv*bv),P(OB+su*bu+sv*bv+6*n))
for ph in (-46,0,46):
  for x in (-27,27):
    print('saddle',ph,x,P(OA+x*ex+34*rad(ph)),P(OA+x*ex+42*rad(ph)))
for xs,ph in [(0,8),(-4,0),(0,20)]:
    S=OA+xs*ex+41*rad(ph);PB=OB+6*n;c=(S-PB)/np.linalg.norm(S-PB)
    print('arm',xs,ph,'S',P(S),'PB',P(PB),'c.U',round(c@U,2),'n.U',round(n@U,2),'-rad.U',round(-rad(ph)@U,2))
