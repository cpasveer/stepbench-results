import sys
sys.argv=['x']
src=open('submission.py').read().split("OUT = os.path.join")[0]
exec(src)
xb = to_a(XB); nb = to_a(NB)
print("xb",xb,"nb",nb,"bcentre",to_a(pb(0,0)))
print("ztop",Z_TOP,Z_BOT)
for x,d in ((-22,0),(22,0),(-22,7),(22,7)): print(x,d,to_a(pb(x,d)))
b_back = to_a(pb(0, T_WALL)); b_front = to_a(pb(0, C_B))
print("i3",meet((T_WALL, U_IN), (1, 0), b_back, xb))
print("o3",meet((C_A, U_OUT), (1, 0), b_front, xb))
