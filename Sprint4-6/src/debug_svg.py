from svgelements import SVG, Shape, Polyline, Path

svg_file = r"C:\Users\Avery\Art-Art-Art-\Sprint4-6\output\person_edge_20260316_111148.svg"
svg = SVG.parse(svg_file)

print(f"Parsing {svg_file}")
count = 0
for element in svg.elements():
    count += 1
    print(f"Element: {type(element)}")
    if isinstance(element, Shape):
        print("  Is Shape: Yes")
        try:
            path = element.as_path()
            print(f"  as_path length: {len(path)}")
        except Exception as e:
            print(f"  as_path error: {e}")
    else:
        print("  Is Shape: No")
        # Check if it has as_path anyway
        if hasattr(element, "as_path"):
             print("  Has as_path: Yes")
        else:
             print("  Has as_path: No")

print(f"Total elements: {count}")
