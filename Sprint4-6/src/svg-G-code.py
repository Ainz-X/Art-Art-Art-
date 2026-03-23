#!/usr/bin/env python3
# -*- coding: utf-8 -*-

"""
svg_to_gcode.py

Convert SVG to simple plotter G-code.

Features:
- Supports path, polyline, polygon, line, rect, circle, ellipse
- Flips Y axis for CNC/plotter-style coordinates
- Scales drawing to a target width/height in mm
- Uses Z moves for pen up / pen down
- Outputs G-code suitable for UGS / GRBL-style senders

Usage:
    python svg_to_gcode.py input.svg output.ngc

Optional arguments are inside CONFIG below.
"""

from __future__ import annotations
import math
import sys
import os
import glob
from dataclasses import dataclass
from typing import Iterable, List, Tuple

from svgelements import SVG, Path, Shape


Point = Tuple[float, float]


@dataclass
class Config:
    # Target size in mm. If one is None, scale is decided by the other.
    target_width_mm: float | None = 120.0
    target_height_mm: float | None = None

    # Margin in mm around the drawing
    margin_mm: float = 5.0

    # Sampling step for curves in SVG units before scaling.
    # Smaller => more points => smoother but slower plotting.
    curve_sample_step: float = 2.0

    # Pen control with Z axis
    pen_up_z: float = 5.0
    pen_down_z: float = 0.0

    # Feed rates
    travel_feed: int = 3000
    draw_feed: int = 1500
    pen_feed: int = 800

    # Safe settings
    absolute_mode: bool = True
    units_mm: bool = True

    # If True, return to origin at end
    home_at_end: bool = True


CONFIG = Config()


def distance(p1: Point, p2: Point) -> float:
    return math.hypot(p2[0] - p1[0], p2[1] - p1[1])


def bbox_of_polylines(
    polylines: List[List[Point]],
) -> Tuple[float, float, float, float]:
    xs = [x for poly in polylines for x, _ in poly]
    ys = [y for poly in polylines for _, y in poly]
    if not xs or not ys:
        raise ValueError("No drawable geometry found in SVG.")
    return min(xs), min(ys), max(xs), max(ys)


def sample_path(shape: Shape, step: float) -> List[Point]:
    """
    Convert a svgelements shape into a list of points.
    """
    pts = []
    
    # CASE 1: Explicit points (Polyline, Polygon)
    if hasattr(shape, 'points'):
        try:
            # svgelements often stores points as complex numbers or Point objects
            raw_pts = shape.points
            if raw_pts:
                for p in raw_pts:
                    # Handle complex numbers (x + yj) commonly used
                    if isinstance(p, complex):
                        pts.append((p.real, p.imag))
                        continue
                    
                    # Handle Point objects or tuples
                    x, y = 0.0, 0.0
                    if hasattr(p, 'x') and hasattr(p, 'y'):
                        x, y = float(p.x), float(p.y)
                    elif isinstance(p, (list, tuple)) and len(p) >= 2:
                        x, y = float(p[0]), float(p[1])
                    elif hasattr(p, 'real') and hasattr(p, 'imag'): # fallback for complex-like
                        x, y = float(p.real), float(p.imag)
                    else:
                        # try indexing
                        try:
                            x, y = float(p[0]), float(p[1])
                        except:
                            continue
                            
                    pts.append((x, y))
                
                if len(pts) > 1:
                    return deduplicate_points(pts)
        except Exception as e:
            print(f"DEBUG: Error reading shape.points: {e}")
            pass

    # CASE 2: Use as_path() for general shapes (Path, Circle, Rect, etc)
    if not hasattr(shape, "as_path"):
        return []

    try:
        path: Path = shape.as_path()
    except Exception as e:
        print(f"DEBUG: as_path() failed: {e}")
        return []

    if len(path) == 0:
        return []

    # Estimate path length for sampling
    try:
        total_len = float(path.length())
    except Exception:
        total_len = 0.0

    if total_len <= 0:
        # fallback: just use endpoints if possible
        pts = []
        for seg in path:
            try:
                pts.append((float(seg.start.real), float(seg.start.imag)))
                pts.append((float(seg.end.real), float(seg.end.imag)))
            except Exception:
                pass
        return deduplicate_points(pts)

    num_samples = max(2, int(math.ceil(total_len / max(step, 0.1))))
    pts: List[Point] = []
    for i in range(num_samples + 1):
        t = i / num_samples
        try:
            p = path.point(t)
            pts.append((float(p.real), float(p.imag)))
        except Exception:
            continue

    return deduplicate_points(pts)


def deduplicate_points(points: List[Point], eps: float = 1e-9) -> List[Point]:
    if not points:
        return points
    result = [points[0]]
    for p in points[1:]:
        if distance(result[-1], p) > eps:
            result.append(p)
    return result


def extract_polylines(svg_file: str, step: float) -> List[List[Point]]:
    # Print what file we are trying to parse
    print(f"Parsing SVG file: {svg_file}")
    
    try:
        svg = SVG.parse(svg_file)
    except Exception as e:
        print(f"FAILED to parse SVG: {e}")
        return []

    polylines: List[List[Point]] = []

    # Iterate over ALL elements to debug
    for i, elem in enumerate(svg.elements()):
        # print(f"Element {i}: {type(elem)}")        
        
        # Check for Shape or specific types we know
        is_shape = isinstance(elem, Shape)
        has_path = hasattr(elem, 'as_path')
        
        # If it's a structural element (Group, SVG, etc), skip unless it has a path
        if not is_shape and not has_path:
            continue

        # Skip invisible
        try:
            if elem.values.get("visibility") == "hidden":
                continue
            if elem.values.get("display") == "none":
                continue
        except Exception:
            pass
            
        # Try to sample
        pts = sample_path(elem, step)
        
        if pts and len(pts) >= 2:
            polylines.append(pts)
        else:
            # print(f"  -> No points extracted from {type(elem)}")
            pass

    print(f"Extracted {len(polylines)} polylines.")
    return polylines


def transform_polylines(polylines: List[List[Point]], cfg: Config) -> List[List[Point]]:
    min_x, min_y, max_x, max_y = bbox_of_polylines(polylines)
    width = max_x - min_x
    height = max_y - min_y

    if width <= 0 or height <= 0:
        raise ValueError("Invalid SVG bounds.")

    # Decide scale
    usable_w = (
        None
        if cfg.target_width_mm is None
        else max(cfg.target_width_mm - 2 * cfg.margin_mm, 1e-6)
    )
    usable_h = (
        None
        if cfg.target_height_mm is None
        else max(cfg.target_height_mm - 2 * cfg.margin_mm, 1e-6)
    )

    if usable_w is not None and usable_h is not None:
        scale = min(usable_w / width, usable_h / height)
    elif usable_w is not None:
        scale = usable_w / width
    elif usable_h is not None:
        scale = usable_h / height
    else:
        scale = 1.0

    transformed: List[List[Point]] = []

    # Convert SVG coordinate system (Y down) to plotter coordinate system (Y up)
    for poly in polylines:
        new_poly: List[Point] = []
        for x, y in poly:
            x0 = (x - min_x) * scale + cfg.margin_mm
            y0 = (max_y - y) * scale + cfg.margin_mm
            new_poly.append((x0, y0))
        transformed.append(new_poly)

    return transformed


def nearest_neighbor_order(polylines: List[List[Point]]) -> List[List[Point]]:
    """
    Simple path ordering to reduce travel moves.
    Reverses polylines when helpful.
    """
    if not polylines:
        return []

    remaining = polylines[:]
    ordered = [remaining.pop(0)]

    while remaining:
        last_end = ordered[-1][-1]
        best_idx = 0
        best_reverse = False
        best_dist = float("inf")

        for i, poly in enumerate(remaining):
            d_start = distance(last_end, poly[0])
            d_end = distance(last_end, poly[-1])

            if d_start < best_dist:
                best_dist = d_start
                best_idx = i
                best_reverse = False

            if d_end < best_dist:
                best_dist = d_end
                best_idx = i
                best_reverse = True

        chosen = remaining.pop(best_idx)
        if best_reverse:
            chosen = list(reversed(chosen))
        ordered.append(chosen)

    return ordered


def gcode_header(cfg: Config) -> List[str]:
    lines = ["; Generated from SVG by svg_to_gcode.py"]
    if cfg.units_mm:
        lines.append("G21 ; set units to millimeters")
    if cfg.absolute_mode:
        lines.append("G90 ; absolute positioning")
    lines.append("G0 Z{:.3f} F{} ; pen up".format(cfg.pen_up_z, cfg.pen_feed))
    return lines


def gcode_footer(cfg: Config) -> List[str]:
    lines = ["G0 Z{:.3f} F{} ; pen up".format(cfg.pen_up_z, cfg.pen_feed)]
    if cfg.home_at_end:
        lines.append("G0 X0.000 Y0.000 F{} ; return to origin".format(cfg.travel_feed))
    lines.append("M2 ; end program")
    return lines


def polylines_to_gcode(polylines: List[List[Point]], cfg: Config) -> List[str]:
    lines = gcode_header(cfg)

    for poly in polylines:
        if len(poly) < 2:
            continue

        start_x, start_y = poly[0]

        # Travel to start with pen up
        lines.append("G0 X{:.3f} Y{:.3f} F{}".format(start_x, start_y, cfg.travel_feed))
        lines.append("G1 Z{:.3f} F{} ; pen down".format(cfg.pen_down_z, cfg.pen_feed))

        # Draw
        for x, y in poly[1:]:
            lines.append("G1 X{:.3f} Y{:.3f} F{}".format(x, y, cfg.draw_feed))

        # Pen up after each polyline
        lines.append("G0 Z{:.3f} F{} ; pen up".format(cfg.pen_up_z, cfg.pen_feed))

    lines.extend(gcode_footer(cfg))
    return lines


def convert_svg_to_gcode(svg_file: str, gcode_file: str, cfg: Config) -> None:
    polylines = extract_polylines(svg_file, cfg.curve_sample_step)
    if not polylines:
        raise ValueError("No drawable paths found in SVG.")

    polylines = transform_polylines(polylines, cfg)
    polylines = nearest_neighbor_order(polylines)

    gcode = polylines_to_gcode(polylines, cfg)

    with open(gcode_file, "w", encoding="utf-8") as f:
        f.write("\n".join(gcode) + "\n")


def main():
    if len(sys.argv) == 3:
        svg_file = sys.argv[1]
        gcode_file = sys.argv[2]
    else:
        # Default to latest SVG in output directory
        print("No arguments provided. Looking for latest SVG in output folder...")
        script_dir = os.path.dirname(os.path.abspath(__file__))
        output_dir = os.path.join(script_dir, "..", "output")
        svg_files = glob.glob(os.path.join(output_dir, "*.svg"))

        if not svg_files:
            print(f"No SVG files found in {output_dir}")
            print("Usage: python svg_to_gcode.py [input.svg] [output.ngc]")
            sys.exit(1)

        # Sort by modification time
        svg_file = max(svg_files, key=os.path.getmtime)
        # Generate output filename in same folder, replacing extension
        gcode_file = os.path.splitext(svg_file)[0] + ".ngc"
        print(f"Auto-selected input: {svg_file}")
        print(f"Auto-selected output: {gcode_file}")

    try:
        convert_svg_to_gcode(svg_file, gcode_file, CONFIG)
        print(f"Done: {gcode_file}")
    except Exception as e:
        print(f"Error: {e}")
        sys.exit(2)


if __name__ == "__main__":
    main()
