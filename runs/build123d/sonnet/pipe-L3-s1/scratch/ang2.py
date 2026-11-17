import runpy,math
g=runpy.run_path('submission.py',run_name='dbg')
D,NB,TG,N,C,O=g['D'],g['NB'],g['TG'],g['N'],g['C'],g['O']
S=C+N*42
best=[]
for a in range(-14,15,4):
  for th in range(-30,31,10):
    t=math.radians(th);rad=NB*math.cos(t)+TG*math.sin(t)
    E=O+D*a+rad*40
    v=(S-E).normalized()          # from landing toward stalk top
    ang=math.degrees(math.acos(v.dot(rad)))
    best.append((round(ang,1),a,th))
best.sort();print(best[:8])
