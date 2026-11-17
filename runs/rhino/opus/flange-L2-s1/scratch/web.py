import math
Ac=(95.0,-91.0); nA=(0.7384553406258838,0.6743023875837234); tA=(-nA[1],nA[0])
Bc=(132.06606660467747,-30.747558502102073); nB=(-0.4786918579404784,-0.8779829754276512); tB=(-nB[1],nB[0])
# check tA matches bolt A5 at u=+30
def fr(P,C,n,t): d=(P[0]-C[0],P[1]-C[1]); return (d[0]*n[0]+d[1]*n[1], d[0]*t[0]+d[1]*t[1])
print('A5',fr((74.77,-68.85),Ac,nA,tA),'B1',fr((118.9,-23.57),Bc,nB,tB))
T=8.0
best=None
for a10 in range(-150,151,5):
  for b10 in range(-150,151,5):
    a=a10/10;b=b10/10
    PA=(Ac[0]+nA[0]*T+tA[0]*a, Ac[1]+nA[1]*T+tA[1]*a)
    PB=(Bc[0]+nB[0]*T+tB[0]*b, Bc[1]+nB[1]*T+tB[1]*b)
    m=0
    worst=1e9
    for i in range(101):
      s=i/100; P=(PA[0]+(PB[0]-PA[0])*s, PA[1]+(PB[1]-PA[1])*s)
      xa,ua=fr(P,Ac,nA,tA); xb,ub=fr(P,Bc,nB,tB)
      if xa<45: worst=min(worst,15-abs(ua))
      if xb<48: worst=min(worst,15-abs(ub))
    if best is None or worst>best[0]: best=(worst,a,b)
print(best)
