from ana import *
import math
a1=np.array([0,-0.573576436,0.819152044]);a2=np.array([0,0.573576436,0.819152044]);ex=np.array([1.,0,0])
w=a2-(a2@N)*N;w/=np.linalg.norm(w)
for d in (0,5,6,8,10,12):
    dr=math.radians(d);u=N*math.cos(dr)+w*math.sin(dr)
    print(d,'u',u.round(3),'a1.u',round(float(a1@u),3),'a2.u',round(float(a2@u),3),'ex.u',round(float(ex@u),3),'U.u',round(float(U@u),2),'V.u',round(float(V@u),2),'D',round(float(np.array([-.44,.09,.89])@u),2))
