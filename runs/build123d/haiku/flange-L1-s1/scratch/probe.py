from build123d import *
import json
T = json.loads(open("task.json").read())
P = T["poses"][0]["placements"]
for k in P:
    print(k, P[k]["origin"])
print(Plane(origin=P["B"]["origin"], x_dir=[P["B"]["rotation"][i][0] for i in range(3)], z_dir=[P["B"]["rotation"][i][2] for i in range(3)]).location)
a = import_step("fixed_A.step"); b = import_step("fixed_B.step")
print("A bbox", a.bounding_box().min, a.bounding_box().max, a.volume)
print("B bbox", b.bounding_box().min, b.bounding_box().max, b.volume)
