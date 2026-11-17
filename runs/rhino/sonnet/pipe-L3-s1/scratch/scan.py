from ana import *
res=[]
for u in [-17,-15,-13]:
 for v in [-6,-3,0,3,6]:
  for xs in [-12,-8,-4,0]:
   for w,h in [(9,9),(12,8)]:
    E=bp(u,v,1.75);S=np.array([xs,22.,32.])
    cl=clearance(S,E,w,h)
    m=min(cl.values()) if cl else 99
    res.append((m,u,v,xs,w,h,{k:round(x,1) for k,x in cl.items() if x<3}))
res.sort(key=lambda r:-r[0])
for r in res[:15]: print(r)
