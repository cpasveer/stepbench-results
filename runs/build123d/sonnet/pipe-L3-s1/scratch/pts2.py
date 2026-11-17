import runpy,math
g=runpy.run_path('submission.py',run_name='dbg')
D,NB,TG,N,C,O=g['D'],g['NB'],g['TG'],g['N'],g['C'],g['O']
E1,E2=g['E1'],g['E2']
for ax in (24.5,):
  for th in (10,20,30,40):
    for R in (34,41):
      t=math.radians(th)
      p=O+D*ax+(NB*math.cos(t)+TG*math.sin(t))*R;q=p-C
      print('axial',ax,'th',th,'R',R,'z',round(p.Z,1),'u,v,n',round(q.dot(E1),1),round(q.dot(E2),1),round(q.dot(N),1))
