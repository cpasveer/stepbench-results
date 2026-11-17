import wall4  # patches cyl
from wall import *
import wall as W
def run(u_step,u_end,w,xt_tall,xt_short):
    E1=bp(u_step,0,0);E2=bp(u_end,0,0)
    Q0=(-4,30)
    a=W.wallcheck(Q0,(E1[1],E1[2]),w,-23.5,xt_tall)
    # short part: from step to end
    b=W.wallcheck((E1[1],E1[2]),(E2[1],E2[2]),w,-23.5,xt_short)
    return {k:round(float(v),1) for k,v in a.items() if v<4},{k:round(float(v),1) for k,v in b.items() if v<4}
for us in (-26,-30,-34):
    for xt in (-4,-6,-8):
        print(us,xt,run(us,-14,12,xt,-12.5))
