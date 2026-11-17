import numpy as np, json
exec(open('scratch/geo.py').read().split("print('B center")[0])
bu=RB[:,0];bv=RB[:,1]
d=0.92*ex+0.004*ey+0.394*ez; d/=np.linalg.norm(d)
U=-d
X=np.cross(U,ex); X/=np.linalg.norm(X); Y=np.cross(U,X)
P=lambda p: np.round([ (np.array(p)-OA)@X,(np.array(p)-OA)@Y,(np.array(p)-OA)@U],1)
print('pipe axis dir in print', np.round([ex@X,ex@Y,ex@U],3))
for b in T['poses'][0]['resolved']['bolts']:
    print(b['id'],P(b['position']),'axis',np.round([np.array(b['axis'])@v for v in (X,Y,U)],2))
for su in (-17,17):
  for sv in (-17,17): print('plate corner',su,sv,P(OB+su*bu+sv*bv))
for ph in (-46,0,46):
  for x in (-26,26):
    f=np.radians(ph); print('saddle',ph,x,P(OA+x*ex+41*(np.sin(f)*ey+np.cos(f)*ez)))
