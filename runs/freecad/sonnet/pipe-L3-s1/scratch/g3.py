import numpy as np, json
T=json.load(open('task.json'))
R=np.array(T['poses'][0]['placements']['A']['rotation']);c=np.array(T['poses'][0]['placements']['A']['origin'])
ax=R[:,0];top=R[:,2];side=np.cross(ax,top)
n=np.array([.3885173161164798,.9214414224885092,0]);ea=np.array([0,0,1.]);eb=np.cross(n,ea)
Bc=np.array([19.549595153015062,49.68566626427389,52.2472790015625])
def W(x,th): 
    t=np.radians(th);return c+x*ax+34*(np.cos(t)*top+np.sin(t)*side)
for x in (-17.5,0,17.5):
  for th in range(-40,41,10):
    N=W(x,th);d=N-Bc
    print(x,th,'eb off',round(d@eb,1),'ea',round(d@ea,1),'n',round(d@n,1))
