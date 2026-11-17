import math, itertools, json
T=json.load(open('task.json'))['poses'][0]['resolved']
nA=(0.7384553406258838,0.6743023875837234); tA=(-nA[1],nA[0])
nB=(-0.4786918579404784,-0.8779829754276512); tB=(nB[1]*-1, nB[0])  # check below
# tB: need t x z = n -> t=(−n_y? ) compute: t=(nB_y*-1?) 
import math
def cross_tz(n): # t such that t x z = n : t=(−n_y... ) t=(a,b): t x z=(b,-a,0)=n -> b=n_x,a=-n_y
    return (-n[1], n[0])
tB=cross_tz(nB); tA=cross_tz(nA)
Ao=(95.0,-91.0); Bo=(132.06606660467747,-30.747558502102073)
def toA(p): d=(p[0]-Ao[0],p[1]-Ao[1]); return (d[0]*tA[0]+d[1]*tA[1], d[0]*nA[0]+d[1]*nA[1])
def toB(p): d=(p[0]-Bo[0],p[1]-Bo[1]); return (d[0]*tB[0]+d[1]*tB[1], d[0]*nB[0]+d[1]*nB[1])
print('B center in A frame',toA(Bo), 'tB in A', (tB[0]*tA[0]+tB[1]*tA[1], tB[0]*nA[0]+tB[1]*nA[1]))
RA=5.0; RB=5.75
TA=4.0; TB=4.6
def forb(p):
    a=toA(p); b=toB(p)
    for k in (-30,-15,15,30):
        if abs(a[0]-k)<RA and TA<=a[1]<=TA+35: return True
    for k in (-15,15):
        if abs(b[0]-k)<RB and TB<=b[1]<=TB+35: return True
    return False
def fromA(t,n): return (Ao[0]+t*tA[0]+n*nA[0], Ao[1]+t*tA[1]+n*nA[1])
def fromB(t,n): return (Bo[0]+t*tB[0]+n*nB[0], Bo[1]+t*tB[1]+n*nB[1])
def inside(poly,p):
    x,y=p;c=False
    for i in range(len(poly)):
        x1,y1=poly[i];x2,y2=poly[(i+1)%len(poly)]
        if (y1>y)!=(y2>y) and x<(x2-x1)*(y-y1)/(y2-y1)+x1: c=not c
    return c
def test(a0,wa,b0,wb):
    # fin quad: A end at (tA=a0,nA=2) width wa along tA ; B end at (tB=b0, nB=2) width wb along tB
    p1=fromA(a0-wa/2,2); p2=fromA(a0+wa/2,2); p3=fromB(b0-wb/2,2); p4=fromB(b0+wb/2,2)
    # order: p1,p2 at A ; p4,p3 ... choose polygon p1,p2,p?; compute both and take convex hull ordering by angle
    pts=[p1,p2,p3,p4]
    cx=sum(p[0] for p in pts)/4; cy=sum(p[1] for p in pts)/4
    pts.sort(key=lambda p: math.atan2(p[1]-cy,p[0]-cx))
    xs=[p[0] for p in pts]; ys=[p[1] for p in pts]
    bad=0
    for i in range(80):
        for j in range(80):
            q=(min(xs)+(max(xs)-min(xs))*i/79, min(ys)+(max(ys)-min(ys))*j/79)
            if inside(pts,q) and forb(q): bad+=1
    return bad
best=[]
for a0 in range(-8,9,2):
  for b0 in range(-8,9,2):
    for wa in (8,10,12,14,16):
      for wb in (8,10,12,14,16):
        if test(a0,wa,b0,wb)==0: best.append((min(wa,wb)+0.01*(wa+wb),a0,wa,b0,wb))
best.sort(reverse=True); print(best[:15])

print('--- allowed interval by nA')
for nAv in range(2,67,3):
    ok=[]
    for k in range(-300,300):
        t=k*0.1
        p=fromA(t,nAv)
        ok.append((t, not forb(p)))
    # contiguous interval containing t around 3
    lo=hi=None
    # find contiguous free run containing t=3
    idx=[i for i,(t,f) in enumerate(ok) if abs(t-3)<0.06]
    i0=idx[0]
    if not ok[i0][1]: print(nAv,'blocked'); continue
    a=i0
    while a>0 and ok[a-1][1]: a-=1
    b=i0
    while b<len(ok)-1 and ok[b+1][1]: b+=1
    print(nAv, round(ok[a][0],1), round(ok[b][0],1), 'tB range', round(toB(fromA(ok[a][0],nAv))[0],1), round(toB(fromA(ok[b][0],nAv))[0],1), 'nB', round(toB(fromA(3,nAv))[1],1))
