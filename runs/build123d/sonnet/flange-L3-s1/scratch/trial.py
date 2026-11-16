import sys, importlib
sys.path.insert(0,'.')
import submission as S
for kv in sys.argv[2:]:
    k,v=kv.split('=')
    setattr(S,k,float(v))
p=S.build()
print('vol',p.volume,'solids',len(p.solids()),'conf',S.tool_conflicts(p))
from build123d import export_step
export_step(p,sys.argv[1])
