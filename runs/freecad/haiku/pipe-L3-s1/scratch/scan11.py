exec(open("/home/user/stepbench-runs/tasks-freecad/pipe-L3-s1/scratch/scan9.py").read().split('print("A:")')[0])
LEGS=[("A",g["LEG_ROOT_A"], P0+r1*g["LEG_END_R"]),("B",g["LEG_ROOT_B"], P0-AX*g["LEG_S3"]+r3*g["LEG_END_R"]),("C",g["LEG_ROOT_C"], P0+AX*g["LEG_S3"]+r3*g["LEG_END_R"])]
names=[]
for bid in BOLT: names+=[bid+"_head",bid+"_tool"]
for nm,ruv,tgt in LEGS:
    root=CB+UB*ruv[0]+ZB*ruv[1]+NB*4.0
    L=bar(root,tgt,11.0)
    hits=[(names[i],round(L.common(kk).Volume,2)) for i,kk in enumerate(keep) if L.common(kk).Volume>0.01]
    print("leg",nm,ruv,"hits",hits)
