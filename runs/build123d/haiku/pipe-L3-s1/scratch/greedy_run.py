import sys, time; sys.path.insert(0,'/home/user/stepbench-runs/tasks/pipe-L3-s1/scratch')
import numpy as np
import greedy as G
from pin import pin_k
from p4 import Anodes, Bnodes, from_B, to_B
# B-local node coordinates
A_loc=np.array([to_B(p) for p in Anodes])
B_loc=np.array([[-12.5,-12.5,2],[12.5,-12.5,2],[12.5,12.5,2],[-12.5,12.5,2]])
grid=[]
for x in [-30,-20,-10,0,10]:
    for y in [-30,-20,-10,0,10]:
        for z in [15,30,45,60]:
            q=np.array([x,y,z],float)
            if G.bolt_dist_ok(q,6): grid.append(q)
I_loc=np.array(grid)
nodes_loc=np.vstack([A_loc,B_loc,I_loc])
NA,NB,NI=4,4,len(I_loc)
N=len(nodes_loc)
print('internal nodes',NI)
# candidate members
cands=[]
for i in range(N):
    for j in range(i+1,N):
        if i<NA and j<NA: continue
        if i<NA+NB and j<NA+NB and i>=NA and j>=NA: # B-B ring only adjacent or diagonal
            pass
        P=nodes_loc[i]; Q=nodes_loc[j]
        if not G.seg_ok(P,Q,1.2): continue
        if not G.pipe_ok(P,Q,1.2): continue
        cands.append((i,j))
print('candidate members',len(cands))
nodes_w=np.array([from_B(q) for q in nodes_loc])
r=1.2; A=np.pi*r*r
chosen=[]
target=30.0
t0=time.time()
cur=0.0
while cur<target and time.time()-t0<900:
    best=None; bestval=-1
    for (i,j) in cands:
        if (i,j) in chosen: continue
        trial=[(a,b,A) for a,b in chosen]+[(i,j,A)]
        k=pin_k(nodes_w,trial,dirs=G.__dict__.get('D',None) or __import__('pin').DIRS[::3])
        L=np.linalg.norm(nodes_w[j]-nodes_w[i])
        val=k/L
        if k>bestval: bestval=k; best=(i,j)
    chosen.append(best); cur=bestval
    print(len(chosen), best, round(bestval,2), flush=True)
print('final chosen', chosen, round(cur,2))
np.save('/home/user/stepbench-runs/tasks/pipe-L3-s1/scratch/chosen.npy', np.array(chosen))
np.save('/home/user/stepbench-runs/tasks/pipe-L3-s1/scratch/nodes_loc.npy', nodes_loc)
