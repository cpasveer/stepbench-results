import sys; sys.path.insert(0,"/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/scratch")
exec(open("/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/scratch/model5.py").read().split("C = xyz(0,0,4)")[0])
def case(label, legs):
    pts=[]; mem=[]; fixed=set(); C=xyz(0,0,4); pts.append(C); rn={}
    for ru,rv,tgt,sec in legs:
        if (ru,rv) not in rn:
            pts.append(xyz(ru,rv,4)); rn[(ru,rv)]=len(pts)-1
            mem.append((0,rn[(ru,rv)],1600.0,1600*400/12.0,2*1600*400/12.0))
        pts.append(tgt); fixed.add(len(pts)-1)
        mem.append((rn[(ru,rv)],len(pts)-1,sec*sec,sec**4/12,0.141*sec**4))
    print(label, minstiff(pts,mem,fixed))
S3=lambda s: P0+AX*s+r3*38.5
case("A11 S1mid, B11 S3-4 from (4,-16), C10 S3+4 from (2,-17.5)", [(1,8,P0+r1*38.5,11),(4,-16,S3(-4),11),(2,-17.5,S3(4),10)])
case("A11 S1mid, B11 S3-6 from (3.5,-16), C11 S3+6 from (-6,0)", [(1,8,P0+r1*38.5,11),(3.5,-16,S3(-6),11),(-6,0,S3(6),11)])
case("A11 S1mid, B11 S3-4 from (4,-16), C11 S3+4 from (0,-17)", [(1,8,P0+r1*38.5,11),(4,-16,S3(-4),11),(0,-17,S3(4),11)])
