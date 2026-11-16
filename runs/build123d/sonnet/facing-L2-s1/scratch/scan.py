import json, math
import numpy as np
B = json.load(open('task.json'))["poses"][0]["resolved"]["bolts"]
A=[np.array(b["position"][:2]) for b in B if b["id"][0]=="A"]
Bb=[np.array(b["position"][:2]) for b in B if b["id"][0]=="B"]
def dseg(p,a,b):
    t=np.clip(np.dot(p-a,b-a)/np.dot(b-a,b-a),0,1); return np.linalg.norm(p-(a+t*(b-a)))
best=[]
for x in range(-150,-100,2):
  for y in range(105,155,2):
    P=np.array([x,y],float)
    m=min([dseg(a,P,b) for a in A for b in Bb]+[dseg(b,P,a) for b in Bb for a in A])
    m2=min(np.linalg.norm(P-q) for q in A+Bb)
    best.append((min(m,m2-10),x,y,m,m2))
best.sort(reverse=True); print(best[:8])
