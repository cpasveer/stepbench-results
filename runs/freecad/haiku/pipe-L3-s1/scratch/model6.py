import sys; sys.path.insert(0,"/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/scratch")
exec(open("/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/scratch/model5.py").read().split("C = xyz(0,0,4)")[0])
def case(label, roots_legs, sec):
    pts=[]; mem=[]; fixed=set()
    C=xyz(0,0,4); pts.append(C)
    rootnodes={}
    for ru,rv,tgt in roots_legs:
        key=(ru,rv)
        if key not in rootnodes:
            pts.append(xyz(ru,rv,4)); rootnodes[key]=len(pts)-1
            mem.append((0,rootnodes[key],1600.0,1600*400/12.0,2*1600*400/12.0))
        pts.append(tgt); fixed.add(len(pts)-1)
        mem.append((rootnodes[key], len(pts)-1, sec*sec, sec**4/12, 0.141*sec**4))
    print(label, "sec",sec, minstiff(pts,mem,fixed))
A_t = P0 + r1*38.5
B_t = P0 - AX*8 + r3*38.5
C_t = P0 + AX*8 + r3*38.5
case("A(1,8)->S1mid, B(3.5,-16)->S3-8, C(-6,8)->S3+8", [(1,8,A_t),(3.5,-16,B_t),(-6,8,C_t)], 10.0)
case("same, 12mm", [(1,8,A_t),(3.5,-16,B_t),(-6,8,C_t)], 12.0)
case("current design (2 legs root(1,8) S1 +-6; root(3.5,-16) S3)", [(1,8,P0-AX*6+r1*38.5),(1,8,P0+AX*6+r1*38.5),(3.5,-16,P0+r3*38.5)], 10.0)
