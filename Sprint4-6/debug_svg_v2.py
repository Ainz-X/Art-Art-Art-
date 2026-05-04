import sys
try:
    from svgelements import SVG, Shape, Path
except ImportError:
    print("Error: svgelements module not found.")
    sys.exit(1)

svg_file = r"C:\Users\Avery\Art-Art-Art-\Sprint4-6\output\person_edge_20260316_111148.svg"
print(f"Reading file: {svg_file}")

try:
    svg = SVG.parse(svg_file)
except Exception as e:
    print(f"Error parsing SVG: {e}")
    sys.exit(1)

print("SVG parsed successfully.")
count = 0
for element in svg.elements():
    count += 1
    print(f"Element {count}: {type(element)}")
    if isinstance(element, Shape):
        print("  Length of path:", len(element.as_path()))
    else:
        print("  Not a Shape instance.")
        if hasattr(element, 'as_path'):
             print("  But has as_path method.")
             
print(f"Total elements: {count}")
