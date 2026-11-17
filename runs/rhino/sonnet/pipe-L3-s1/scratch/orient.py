from ana import *
import math
a1=np.array([0,-0.573576436,0.819152044]);a2=np.array([0,0.573576436,0.819152044]);ex=np.array([1.,0,0])
tg1=np.cross(a1,ex);tg2=np.cross(a2,ex)
# arm direction approx
base=np.array([1,0,0])+36.5*a2; Tm=bp(-25,-15,2.25);D=Tm-base;D/=np.linalg.norm(D)
normals={}
normals['plate_back']=-N;normals['plate_front']=N
normals['tab_back']=-N
for nm,a,tg in (('p1',a1,tg1),('p2',a2,tg2)):
    normals[nm+'_top']=a;normals[nm+'_end+']=ex;normals[nm+'_end-']=-ex;normals[nm+'_s+']=tg;normals[nm+'_s-']=-tg
for al in range(-46,47,6):
    r=np.array([0,math.sin(math.radians(al)),math.cos(math.radians(al))])
    normals['seat%d'%al]=-r; normals['bridge%d'%al]=r
# arm sides
e1=np.cross(D,ex);e1/=np.linalg.norm(e1)
# oblique prism sides: planes contain D and edge directions ex / tg2(base rect edges)
for nm,ed in (('ex',ex),('tg',tg2)):
    nn=np.cross(D,ed);nn/=np.linalg.norm(nn)
    normals['arm_'+nm+'+']=nn;normals['arm_'+nm+'-']=-nn
def worst(u,exclude=()):
    w=-99;wn=None
    for k,n in normals.items():
        if n@u<-0.999 : continue  # bed
        ov=90-math.degrees(math.acos(max(-1,min(1,-(n@u)))))  # normal down -> overhang 90
        # normal exactly down => 90; normal horizontal => 0
        ov=math.degrees(math.asin(max(-1,min(1,-(n@u)))))
        if ov>w: w=ov;wn=k
    return w,wn
best=[]
for th in np.linspace(0,math.pi,91):
  for ph in np.linspace(0,2*math.pi,180,endpoint=False):
    u=np.array([math.sin(th)*math.cos(ph),math.sin(th)*math.sin(ph),math.cos(th)])
    w,wn=worst(u)
    best.append((w,wn,u))
best.sort(key=lambda t:t[0])
for b in best[:8]: print(round(b[0],1),b[1],b[2].round(2))
print('N',N.round(2),'D',D.round(2))
