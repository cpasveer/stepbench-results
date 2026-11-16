import sys
sys.argv=['submission']
exec(open('submission.py').read().split("OUT = os.path")[0])
for p in [V(36.934214207883, 102.33999621464, 13.195397230635),V(19.814433980851, 101.40041732502, 16.49277560297)]:
    l=to_local_pt(p); print(l, 'tv', l.y*S.y+l.z*S.z, l.dot(BV)-BCV, 'r',(l.y**2+l.z**2)**.5)
