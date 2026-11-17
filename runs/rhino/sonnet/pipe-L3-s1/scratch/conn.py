from fe2 import *
import collections
def components(occ):
    idx=np.argwhere(occ);S=set(map(tuple,idx));seen=set();comps=[]
    for v in S:
        if v in seen: continue
        q=collections.deque([v]);seen.add(v);c=0
        while q:
            x=q.popleft();c+=1
            for d in ((1,0,0),(-1,0,0),(0,1,0),(0,-1,0),(0,0,1),(0,0,-1)):
                y=(x[0]+d[0],x[1]+d[1],x[2]+d[2])
                if y in S and y not in seen: seen.add(y);q.append(y)
        comps.append(c)
    return sorted(comps,reverse=True)
if __name__=='__main__':
    g=Grid(1.0);base=default_params();base.update(XB=-20.5,WALL_W=14.0,ROOT_W=14.0,T=9.0,PAD_HW=5.0,TOP_W=0.0,SLOPE=0.3)
    occ=build(g,base);print(components(occ)[:5])
