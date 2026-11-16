import sys; sys.path.insert(0,'.')
import submission as S
from build123d import *
p=S.build()
print('extra0', S.tool_conflicts(p,extra=0.0), 'extra0.15', S.tool_conflicts(p,extra=0.15))
