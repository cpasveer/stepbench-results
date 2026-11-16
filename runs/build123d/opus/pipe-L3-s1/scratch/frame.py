import numpy as np, json
T=json.load(open('task.json'));P=T['poses'][0]
d=np.array([0.2487956429525063,0.5900653629615055,0.7680648380709507])
n=np.array([0.3885173161164798,0.9214414224885092,0])
c1=np.array([0.9214414224885092,-0.3885173161164798,0])
e=(n-np.dot(n,d)*d);e/=np.linalg.norm(e)
print('n.d',n@d,'e',e,'c1.d',c1@d, 'c1.e',c1@e)
Bc=np.array(P['resolved']['mounting_faces'][0]['origin'])
C=np.array([21,136.5,-7.])
def fr(p): q=np.array(p)-Bc; return np.round([q@e,q@c1,q@d],2)
print('pipe C',fr(C))
for b in P['resolved']['bolts']:
    print(b['id'],fr(b['position']), np.round([np.array(b['axis'])@e,np.array(b['axis'])@c1,np.array(b['axis'])@d],3))
