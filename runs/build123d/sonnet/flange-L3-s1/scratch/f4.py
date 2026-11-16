from build123d import *
for n,idx in (('0003',22),('0002',43),('0001',3)):
    p=import_step('attempts/%s/output.step'%n)
    fs=p.faces()
    areas=sorted([(round(f.area,1),i,str(f.geom_type).split('.')[-1]) for i,f in enumerate(fs) if 'PLANE' in str(f.geom_type)],reverse=True)
    print(n,len(fs),'chosen',idx,round(fs[idx].area,1))
    print(areas[:16])
    print([a for a in areas if a[1]==idx], 'rank',[k for k,a in enumerate(areas) if a[1]==idx])
