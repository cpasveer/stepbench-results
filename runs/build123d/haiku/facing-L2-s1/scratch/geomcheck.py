from build123d import import_step
p = import_step("/home/user/stepbench-runs/tasks/facing-L2-s1/output.step")
print("solids", len(p.solids()), "valid", p.is_valid, "volume", round(p.volume, 1))
bb = p.bounding_box(); print("bbox", bb.min, bb.max)
