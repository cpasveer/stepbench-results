from build123d import import_step, Vector
import json
p = import_step("output.step")
print("solids", len(p.solids()), "valid", p.is_valid, "volume", round(p.volume,1))
bb = p.bounding_box(); print("bbox", bb.min, bb.max)
A = import_step("fixed_A.step"); B = import_step("fixed_B.step")
ov_A = p.intersect(A).volume; ov_B = p.intersect(B).volume
print("overlap A", ov_A, "overlap B", ov_B)
print("fixed A bbox", A.bounding_box().min, A.bounding_box().max)
print("fixed B bbox", B.bounding_box().min, B.bounding_box().max)
