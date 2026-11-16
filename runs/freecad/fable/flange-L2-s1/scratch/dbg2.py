src=open('submission.py').read().split("OUT = os.path.join")[0]
exec(src)
import Part
s=Part.read('output.step')
# head clearance cylinders
for b in BOLTS:
    ax=V(*b['axis']); pos=V(*b['position'])
    r=b['head_diameter']/2+0.5
    c=Part.makeCylinder(r,b['head_height']+1,pos+ax*T_WALL,ax)
    print(b['id'],'headspace overlap',s.common(c).Volume, 'dist', s.distToShape(Part.makeCylinder(b['head_diameter']/2,b['head_height'],pos+ax*(T_WALL+0.01),ax))[0])
