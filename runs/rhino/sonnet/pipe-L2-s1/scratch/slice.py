import numpy as np
np.set_printoptions(suppress=True,precision=2)
n=lambda v:np.array(v)/np.linalg.norm(v)
PO=np.array([21,136.5,-7.]);PD=n([0.2487956429525063,0.5900653629615055,0.7680648380709507])
R1=n([0.7093118905965579,-0.6509870200868549,0.27035632165831364]);R2=n([-0.3771547728198181,-0.6713858946506742,0.6379617989560074])
BN=n([0.3885173161164798,0.9214414224885092,0]);BW=np.array([0,0,1.]);BU=n(np.cross(BW,BN))
BO=np.array([19.549595153015062,49.68566626427389,52.2472790015625])
print('BU',BU,'BU.PD',BU@PD)
Wp=np.cross(BU,PD)  # t axis
def sl(v): return np.array([v@BU, v@Wp])
BIS=n(R1+R2)
print('b',sl(BIS),'r1',sl(R1),'r2',sl(R2))
cosa=BIS@R1;RT=40
V=BIS*RT/cosa;print('ridge',sl(V))
def away(ra,rb):
    t=rb-(rb@ra)*ra;t/=np.linalg.norm(t);return -t
E1=R1*RT+away(R1,R2)*9;E2=R2*RT+away(R2,R1)*9
print('E1',sl(E1),'E2',sl(E2))
print('bolt A1',sl(R1*34),'A3',sl(R2*34))
print('plate center',sl(BO-PO),'plate corners', [sl(BO-PO+BU*x)  for x in (-17.8,17.8)])
print('axial plate',(BO-PO)@PD)
