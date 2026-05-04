import argparse
import subprocess
import time
from pathlib import Path

import cv2
import numpy as np

try:
    from ultralytics import YOLO
except ImportError:
    print("Error: ultralytics was not found.")
    raise SystemExit(1)


def build_arg_parser():
    parser = argparse.ArgumentParser(description="YOLO -> SVG -> G-code -> Plotter")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--output", type=str, default="output/pi_draw.svg")
    parser.add_argument("--model", type=str, default="yolov8n-seg.pt")
    parser.add_argument("--conf", type=float, default=0.25)
    parser.add_argument("--auto-trigger-seconds", type=float, default=2.0)
    parser.add_argument("--pose-seconds", type=float, default=5.0)
    parser.add_argument("--target-width-mm", type=float, default=210.0)
    parser.add_argument("--target-height-mm", type=float, default=210.0)
    parser.add_argument("--max-width-mm", type=float, default=400.0)
    parser.add_argument("--max-height-mm", type=float, default=200.0)
    parser.add_argument("--draw-feed", type=int, default=1500)
    parser.add_argument("--travel-feed", type=int, default=3000)
    
    parser.add_argument("--pen-up-servo", type=int, default=100)
    parser.add_argument("--pen-down-servo", type=int, default=500)
    parser.add_argument(
        "--plot-offset-x-mm",
        type=float,
        default=10.0,
        help="Shift whole plot right in mm (away from left limit)",
    )
    parser.add_argument("--mirror", action="store_true", default=True)
    parser.add_argument("--no-mirror", action="store_false", dest="mirror")
    parser.add_argument("--edge-only", action="store_true", default=True)
    parser.add_argument("--no-edge-only", action="store_false", dest="edge_only")
    parser.add_argument("--home", action="store_true", default=True)
    parser.add_argument("--no-home", action="store_false", dest="home")
    parser.add_argument("--no-gui", action="store_true")
    return parser


def save_svg(contours, width, height, output_path):
    output_path.parent.mkdir(parents=True, exist_ok=True)
    lines = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">']
    for contour in contours:
        points = " ".join(f"{int(x)},{int(y)}" for x, y in contour)
        lines.append(f'  <polyline points="{points}" fill="none" stroke="black" stroke-width="2"/>')
    lines.append("</svg>")
    output_path.write_text("\n".join(lines), encoding="utf-8")


def simplify_contour(contour, epsilon_ratio):
    if contour.shape[0] < 3 or epsilon_ratio <= 0:
        return contour
    perimeter = cv2.arcLength(contour, True)
    return cv2.approxPolyDP(contour, perimeter * epsilon_ratio, True).reshape((-1, 2))


def render_preview(frame, results, edge_only):
    if not edge_only:
        return frame.copy()
    canvas = np.zeros_like(frame)
    if results.masks is None:
        return canvas
    masks = results.masks.data.cpu().numpy()
    mask = (masks[0] > 0.5).astype(np.uint8) * 255
    mask = cv2.resize(mask, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_NEAREST)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    if contours:
        cv2.drawContours(canvas, contours, -1, (255, 255, 255), 2)
    return canvas


def run_pipeline(args):
    model = YOLO(args.model)
    cap = cv2.VideoCapture(args.camera)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    if not cap.isOpened():
        raise RuntimeError("Unable to open the camera.")

    win_name = "AI Art System"
    if not args.no_gui:
        cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win_name, 1280, 720)

    person_seen_start = None
    pose_deadline = None
    print("Vision-enhanced mode is ready. Running...")

    while True:
        ok, frame = cap.read()
        if not ok:
            break

        if args.mirror:
            frame = cv2.flip(frame, 1)

        results = model.predict(source=frame, conf=args.conf, verbose=False)[0]
        preview = render_preview(frame, results, args.edge_only)
        now = time.time()
        has_detection = len(results.boxes) > 0

        if pose_deadline:
            left = pose_deadline - now
            if left > 0:
                cv2.putText(preview, f"CAPTURING: {left:.1f}s", (50, 100), 2, 2, (0, 255, 255), 3)
            else:
                print("📸 Capturing photo...")
                timestamp = time.strftime("%Y%m%d_%H%M%S")
                target_svg = Path(args.output).with_name(f"pi_draw_{timestamp}.svg")
                target_png = target_svg.with_suffix(".png")

                found_shape = False
                if results.masks is not None:
                    masks = results.masks.data.cpu().numpy()
                    raw_mask = (masks[0] > 0.5).astype(np.uint8) * 255
                    mask = cv2.resize(raw_mask, (frame.shape[1], frame.shape[0]), interpolation=cv2.INTER_NEAREST)
                    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    if contours:
                        best_contour = max(contours, key=cv2.contourArea).squeeze()
                        save_svg([simplify_contour(best_contour, 0.002)], frame.shape[1], frame.shape[0], target_svg)
                        cv2.imwrite(str(target_png), preview)
                        found_shape = True

                if not found_shape:
                    print("Could not capture a clear contour, resetting.")
                    pose_deadline = None
                    person_seen_start = None
                    continue

                cap.release()
                cv2.destroyAllWindows()
                print(f"SVG generated: {target_svg}")
                target_gcode = target_svg.with_suffix(".ngc")

                svg_cmd = [
                    "python3",
                    "svg-G-code.py",
                    str(target_svg),
                    str(target_gcode),
                    "--target-width-mm",
                    str(args.target_width_mm),
                    "--target-height-mm",
                    str(args.target_height_mm),
                    "--max-width-mm",
                    str(args.max_width_mm),
                    "--max-height-mm",
                    str(args.max_height_mm),
                    "--draw-feed",
                    str(args.draw_feed),
                    "--travel-feed",
                    str(args.travel_feed),
                    "--pen-up-servo",
                    str(args.pen_up_servo),
                    "--pen-down-servo",
                    str(args.pen_down_servo),
                    "--plot-offset-x-mm",
                    str(args.plot_offset_x_mm),
                ]

                print("Automatically converting G-code...")
                subprocess.run(svg_cmd, check=True)

                print("Starting the plotter...")
                send_cmd = ["python3", "send.py", str(target_gcode)]
                if args.home:
                    send_cmd.append("--home")
                subprocess.run(send_cmd, check=True)

                print("Plotting has started, exiting.")
                return {"svg": str(target_svg), "gcode": str(target_gcode), "preview": str(target_png)}
        elif has_detection:
            if person_seen_start is None:
                person_seen_start = now
            if (now - person_seen_start) >= args.auto_trigger_seconds:
                pose_deadline = now + args.pose_seconds
            else:
                cv2.putText(preview, "LOCKED", (50, 100), 2, 2, (0, 255, 0), 3)
        else:
            person_seen_start = None
            cv2.putText(preview, "READY", (50, 100), 2, 2, (255, 255, 255), 2)

        if not args.no_gui:
            cv2.imshow(win_name, preview)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break

    cap.release()
    cv2.destroyAllWindows()
    raise RuntimeError("The vision pipeline ended early and produced no output.")


def main():
    args = build_arg_parser().parse_args()
    run_pipeline(args)


if __name__ == "__main__":
    main()
