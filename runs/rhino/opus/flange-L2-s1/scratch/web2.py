import math
Ac=(95.0,-91.0); nA=(0.7384553406258838,0.6743023875837234); tA=(-nA[1],nA[0])
Bc=(132.06606660467747,-30.747558502102073); nB=(-0.4786918579404784,-0.8779829754276512); tB=(-nB[1],nB[0])
def fr(P,C,n,t): d=(P[0]-C[0],P[1]-C[1]); return (d[0]*n[0]+d[1]*n[1], d[0]*t[0]+d[1]*t[1])
T=8.0
for ht in (3.0,3.5,4.0):
  best=None
  for a10 in range(-150,151,2):
    for b10 in range(-150,151,2):
      a=a10/10;b=b10/10
      PA=(Ac[0]+nA[0]*T+tA[0]*a, Ac[1]+nA[1]*T+tA[1]*a)
      PB=(Bc[0]+nB[0]*T+tB[0]*b, Bc[1]+nB[1]*T+tB[1]*b)
      worst=1e9
      for i in range(201):
        s=i/200; P=(PA[0]+(PB[0]-PA[0])*s, PA[1]+(PB[1]-PA[1])*s)
        xa,ua=fr(P,Ac,nA,tA); xb,ub=fr(P,Bc,nB,tB)
        d=15-abs(ua)-ht
        if xa<16: worst=min(worst,d-4.75)
        elif xa<45: worst=min(worst,d-2.5)
        d=15-abs(ub)-ht
        if xb<16: worst=min(worst,d-5.5)
        elif xb<48: worst=min(worst,d-2.5)
      if best is None or worst>best[0]: best=(round(worst,2),a,b)
  print(ht,best)
