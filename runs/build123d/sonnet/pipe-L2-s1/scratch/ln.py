import sys; sys.path.insert(0,'.')
from submission import *
for k in ["B1","B2","B3","B4"]:
    p=loc(BOLTS[k]["position"])
    for t in (50,60,70,80,90,100): print(k,t,np.round(p+BN*t,1))
