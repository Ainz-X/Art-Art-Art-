import argparse
import time
from pathlib import Path

import cv2
import numpy as np

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Use YOLO to detect person contour and export a single-line SVG."
    )
    parser.add_argument("--camera", type=int, default=0, help="Camera index")
    parser.add_argument("--width", type=int, default=1280, help="Capture width")
    parser.add_argument("--height", type=int, default=720, help="Capture height")
    parser.add_argument("--output", type=str, default="output/person_edge.svg", help="Output SVG path")
    parser.add_argument("--epsilon", type=float, default=0.01, help="Contour simplify ratio")
    parser.add_argument("--model", type=str, default="yolov8n-seg.pt", help="YOLO segmentation model path")
    parser.add_argument("--conf", type=float, default=0.35, help="YOLO confidence threshold")
    parser.add_argument("--imgsz", type=int, default=640, help="YOLO inference size")
    parser.add_argument("--smooth-iters", type=int, default=2, help="Chaikin smoothing iterations")
    parser.add_argument("--resample-step", type=float, default=3.0, help="Target pixel step for SVG points")
    parser.add_argument(
        "--min-area-ratio",
        type=float,
        default=0.003,
        help="Minimum contour area ratio against full frame",
    )
    return parser


def simplify_contour(contour: np.ndarray, epsilon_ratio: float) -> np.ndarray:
    if contour.shape[0] < 3:
        return contour

    contour_cv = contour.reshape((-1, 1, 2)).astype(np.int32)
    peri = cv2.arcLength(contour_cv, True)
    epsilon = max(0.5, peri * epsilon_ratio)
    return cv2.approxPolyDP(contour_cv, epsilon, True).reshape((-1, 2))


def chaikin_smooth(contour: np.ndarray, iterations: int) -> np.ndarray:
    if contour.shape[0] < 3 or iterations <= 0:
        return contour

    pts = contour.astype(np.float32)
    for _ in range(iterations):
        n = pts.shape[0]
        smoothed = []
        for i in range(n):
            p0 = pts[i]
            p1 = pts[(i + 1) % n]
            q = 0.75 * p0 + 0.25 * p1
            r = 0.25 * p0 + 0.75 * p1
            smoothed.append(q)
            smoothed.append(r)
        pts = np.asarray(smoothed, dtype=np.float32)
    return np.rint(pts).astype(np.int32)


def resample_closed_contour(contour: np.ndarray, step: float) -> np.ndarray:
    if contour.shape[0] < 3 or step <= 0:
        return contour

    pts = contour.astype(np.float32)
    closed = np.vstack([pts, pts[0]])
    seg_vecs = closed[1:] - closed[:-1]
    seg_lens = np.linalg.norm(seg_vecs, axis=1)
    perimeter = float(np.sum(seg_lens))
    if perimeter <= 1e-6:
        return contour

    sample_count = max(16, int(perimeter / step))
    distances = np.linspace(0.0, perimeter, sample_count, endpoint=False)

    resampled = []
    seg_idx = 0
    seg_start = 0.0

    for d in distances:
        while seg_idx < len(seg_lens) - 1 and seg_start + seg_lens[seg_idx] < d:
            seg_start += seg_lens[seg_idx]
            seg_idx += 1

        seg_len = max(seg_lens[seg_idx], 1e-6)
        t = (d - seg_start) / seg_len
        p = closed[seg_idx] + t * seg_vecs[seg_idx]
        resampled.append(p)

    return np.rint(np.asarray(resampled, dtype=np.float32)).astype(np.int32)


def beautify_contour(contour: np.ndarray, epsilon_ratio: float, smooth_iters: int, resample_step: float) -> np.ndarray:
    simplified = simplify_contour(contour, epsilon_ratio)
    smoothed = chaikin_smooth(simplified, smooth_iters)
    return resample_closed_contour(smoothed, resample_step)


def save_svg(contour: np.ndarray, width: int, height: int, output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    points = " ".join(f"{int(x)},{int(y)}" for x, y in contour)
    svg_text = (
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}">\n'
        f'  <polyline points="{points}" fill="none" stroke="black" stroke-width="2" '
        f'stroke-linejoin="round" stroke-linecap="round" />\n'
        "</svg>\n"
    )
    output_path.write_text(svg_text, encoding="utf-8")


def extract_person_contour_yolo(
    frame: np.ndarray,
    model,
    conf: float,
    imgsz: int,
    min_area_ratio: float,
) -> tuple[np.ndarray | None, tuple[int, int, int, int] | None, str]:
    result = model.predict(
        source=frame,
        conf=conf,
        imgsz=imgsz,
        classes=[0],
        verbose=False,
        device="cpu",
    )[0]

    frame_h, frame_w = frame.shape[:2]
    min_area = float(frame_h * frame_w) * min_area_ratio

    if result.masks is not None and result.boxes is not None and len(result.boxes) > 0:
        masks = result.masks.data.cpu().numpy()
        boxes_xyxy = result.boxes.xyxy.cpu().numpy().astype(np.int32)
        scores = result.boxes.conf.cpu().numpy()

        best_item = None
        for idx, raw_mask in enumerate(masks):
            mask = (raw_mask > 0.5).astype(np.uint8) * 255
            if mask.shape[0] != frame_h or mask.shape[1] != frame_w:
                mask = cv2.resize(mask, (frame_w, frame_h), interpolation=cv2.INTER_NEAREST)

            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=1)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)

            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_NONE)
            if not contours:
                continue

            contour = max(contours, key=cv2.contourArea)
            area = float(cv2.contourArea(contour))
            if area < min_area:
                continue

            weight = area * float(scores[idx])
            x1, y1, x2, y2 = boxes_xyxy[idx]
            bbox = (int(x1), int(y1), int(max(1, x2 - x1)), int(max(1, y2 - y1)))
            if best_item is None or weight > best_item[0]:
                best_item = (weight, contour, bbox)

        if best_item is None:
            return None, None, "YOLO found person but mask is too small"

        contour = best_item[1].squeeze(axis=1)
        if contour.ndim != 2 or contour.shape[0] < 3:
            return None, None, "Invalid contour from segmentation"
        return contour.astype(np.int32), best_item[2], "Person segmented"

    if result.boxes is not None and len(result.boxes) > 0:
        boxes_xyxy = result.boxes.xyxy.cpu().numpy().astype(np.int32)
        scores = result.boxes.conf.cpu().numpy()
        best_idx = int(np.argmax(scores))
        x1, y1, x2, y2 = boxes_xyxy[best_idx]
        bw = max(1, x2 - x1)
        bh = max(1, y2 - y1)
        bbox_area = float(bw * bh)
        if bbox_area < min_area:
            return None, None, "YOLO box too small"

        contour = np.array(
            [[x1, y1], [x2, y1], [x2, y2], [x1, y2]],
            dtype=np.int32,
        )
        return contour, (int(x1), int(y1), int(bw), int(bh)), "Using YOLO bounding box"

    return None, None, "No person detected"


def main() -> None:
    args = build_arg_parser().parse_args()

    if YOLO is None:
        raise RuntimeError("ultralytics is not installed. Run: pip install ultralytics")

    model = YOLO(args.model)

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera index {args.camera}.")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    output_path = Path(args.output)
    last_saved_path = ""
    last_status = "Waiting for detection"

    print("Camera started.")
    print("Press S to capture SVG, Q to quit.")
    print(f"YOLO model: {args.model}")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Failed to read frame from camera.")
                break

            contour, roi, status = extract_person_contour_yolo(
                frame,
                model,
                args.conf,
                args.imgsz,
                args.min_area_ratio,
            )
            last_status = status

            preview = frame.copy()
            if roi is not None:
                x, y, bw, bh = roi
                cv2.rectangle(preview, (x, y), (x + bw, y + bh), (255, 180, 0), 2)

            if contour is not None:
                cv2.polylines(preview, [contour.reshape((-1, 1, 2))], True, (0, 255, 0), 2)

            line1 = f"Status: {last_status}"
            line2 = f"Last SVG: {last_saved_path}" if last_saved_path else "Last SVG: none"
            cv2.putText(preview, line1, (20, 35), cv2.FONT_HERSHEY_SIMPLEX, 0.75, (0, 255, 255), 2, cv2.LINE_AA)
            cv2.putText(preview, line2, (20, 65), cv2.FONT_HERSHEY_SIMPLEX, 0.65, (0, 220, 220), 2, cv2.LINE_AA)

            cv2.imshow("YOLO Person Edge -> SVG", preview)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("s"):
                if contour is None:
                    print(f"Save blocked: {last_status}")
                    continue

                refined = beautify_contour(contour, args.epsilon, args.smooth_iters, args.resample_step)
                if refined.shape[0] < 3:
                    print("Save blocked: contour too small after simplify")
                    continue

                timestamp = time.strftime("%Y%m%d_%H%M%S")
                target = output_path.with_name(f"{output_path.stem}_{timestamp}.svg")
                h, w = frame.shape[:2]
                save_svg(refined, w, h, target)
                last_saved_path = str(target)
                print(f"Saved SVG: {target}")

    finally:
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
