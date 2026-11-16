import sys; sys.path.insert(0,'.')
from submission import *
print("BU",BU,"BV",BV,"BN",BN,"BO",BO)
E=BO+BN*(PLATE_T+L1); S=np.array([SX,0,SZ]); d2=unit(S-E)
print("E",E,"d2",d2,"d2 in plate uvn",d2@BU,d2@BV,d2@BN)
print("E uv", (E-BO)@BU,(E-BO)@BV)
for k in ["B1","B2","B3","B4"]:
    p=loc(BOLTS[k]["position"]); print(k,(p-BO)@BU,(p-BO)@BV)
