import sys; sys.path.insert(0,"/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/scratch")
exec(open("/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/scratch/model7.py").read().split("S3=lambda")[0])
S3=lambda s: P0+AX*s+r3*38.5
S1=lambda s: P0+AX*s+r1*38.5
case("S1 12 (2,-2)+6, S1 12 (0,-11)-6, S3 9 (4,-14)-2", [(2,-2,S1(6),12),(0,-11,S1(-6),12),(4,-14,S3(-2),9)])
case("S1 12 (2,-2)+6, S1 12 (0,-11)-6, S3 11 (4,-16)-2 (reference)", [(2,-2,S1(6),12),(0,-11,S1(-6),12),(4,-16,S3(-2),11)])
case("S1 11 (2,-2)+6, S1 11 (0,-11)-6, S3 9 (4,-14)-2", [(2,-2,S1(6),11),(0,-11,S1(-6),11),(4,-14,S3(-2),9)])
case("S1 12 (2,-2)+6, S1 12 (0,-11)-6, S3 8 (4,-13)-2", [(2,-2,S1(6),12),(0,-11,S1(-6),12),(4,-13,S3(-2),8)])
