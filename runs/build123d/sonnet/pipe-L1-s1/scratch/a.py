import sys;sys.path.insert(0,'.')
import numpy as np, submission as s
u=s.unit
nb=u(s.FACE['normal']);cb=np.array(s.FACE['origin']);zh=np.array([0,0,1.])
n1=u(s.BOLTS['A1']['axis']);n2=u(s.BOLTS['A3']['axis']);m=u(n1+n2)
S=cb+3*nb-14.5*zh;E=s.A0+36*m;d=u(E-S)
print(np.degrees(np.arccos(d@nb)),np.degrees(np.arccos(d@m)), S, E)
