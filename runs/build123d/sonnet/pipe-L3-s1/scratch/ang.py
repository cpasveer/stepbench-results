import runpy,math
g=runpy.run_path('submission.py',run_name='dbg')
D,NB,N,C,O,E=g['D'],g['NB'],g['N'],g['C'],g['O'],g['E']
for HT in (42,50,58,66):
    S=C+N*HT
    v=(E-S).normalized()
    print(HT,'bar vs NB',round(math.degrees(math.acos(v.dot(NB))),1),'len',round((E-S).length,1),'S->E n-comp',round((E-C).dot(N),1))
print('E-C n',(E-C).dot(N))
