import runpy,math
g=runpy.run_path('submission.py',run_name='dbg')
D,NB,TG,N,C,O=g['D'],g['NB'],g['TG'],g['N'],g['C'],g['O']
E1,E2=g['E1'],g['E2']
for ax in (24.5,17.5,0,-24.5):
  for th in (-45,0,45):
    t=math.radians(th);R=41
    p=O+D*ax+(NB*math.cos(t)+TG*math.sin(t))*R
    q=p-C
    print('axial',ax,'th',th,'z',round(p.Z,1),'rel plate u,v,n',round(q.dot(E1),1),round(q.dot(E2),1),round(q.dot(N),1))
print('TG.z',TG.Z,'NB.z',NB.Z)
