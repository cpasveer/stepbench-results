import numpy as np,sys
n=lambda v:np.array(v)/np.linalg.norm(v)
PO=np.array([21,136.5,-7.]);PD=n([.25,.59,.77])
R1=n([0.7093118905965579,-0.6509870200868549,0.27035632165831364]);R2=n([-0.3771547728198181,-0.6713858946506742,0.6379617989560074])
BIS=n(R1+R2)
BO=np.array([19.549595153015062,49.68566626427389,52.2472790015625]);BN=n([0.3885173161164798,0.9214414224885092,0]);BW=np.array([0,0,1.]);BU=np.cross(BW,BN)
bw=lambda x,y,z:BO+BU*x+BW*y+BN*z
def run(bx,by,z0,rho):
    base=[bw(bx[0],by[0],z0),bw(bx[1],by[0],z0),bw(bx[1],by[1],z0),bw(bx[0],by[1],z0)]
    bc=np.mean(base,axis=0);Vv=PO+BIS*rho-bc
    for p in base:
        q=p+Vv-PO;ax=q@PD;rad=q-PD*ax
        print(round(rad@BIS,1),round(rad@R1,1),round(rad@R2,1),round(ax,1),round(np.linalg.norm(rad),1))
run((-6,6),(-17,-9),2,41)
