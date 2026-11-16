__file__='/home/user/stepbench-runs/tasks-freecad/pipe-L1-s1/submission.py'
import sys
sys.argv=['x']
exec(open(__file__).read().split('OUT = ')[0])
s=Part.read('output.step')
for vx in s.Vertexes:
    p=vx.Point; q=p-OB; r=p-OA
    print('B(u,v,n)=(%.1f,%.1f,%.1f)  A(x,y,z)=(%.1f,%.1f,%.1f) rA=%.1f'%(q.dot(BU),q.dot(BV),q.dot(BN),r.dot(AX),r.dot(AY),r.dot(AZ),(r.dot(AY)**2+r.dot(AZ)**2)**.5))
