import Part, collections
s=Part.read("output.step"); print(collections.Counter(f.Surface.__class__.__name__ for f in s.Faces))
