import argparse
import time
from pathlib import Path
from urllib.request import urlretrieve

import cv2
import numpy as np

try:
    from ultralytics import YOLO
except ImportError:
    YOLO = None

try:
    import mediapipe as mp
except ImportError:
    mp = None


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Use YOLO to detect person contours and export an SVG."
    )
    parser.add_argument("--camera", type=int, default=0, help="Camera index")
    parser.add_argument("--width", type=int, default=1280, help="Capture width")
    parser.add_argument("--height", type=int, default=720, help="Capture height")
    parser.add_argument("--output", type=str, default="output/person_edge.svg", help="Output SVG path")
    parser.add_argument("--epsilon", type=float, default=0.0, help="Contour simplify ratio (0 keeps raw contour)")
    parser.add_argument("--model", type=str, default="yolov8n-seg.pt", help="YOLO segmentation model path")
    parser.add_argument("--conf", type=float, default=0.35, help="YOLO confidence threshold")
    parser.add_argument("--imgsz", type=int, default=640, help="YOLO inference size")
    parser.add_argument(
        "--max-people",
        type=int,
        default=0,
        help="Max number of people to keep (0 keeps all detections)",
    )
    parser.add_argument("--smooth-iters", type=int, default=0, help="Chaikin smoothing iterations")
    parser.add_argument("--resample-step", type=float, default=0.0, help="Target pixel step for SVG points (0 disables resampling)")
    parser.add_argument("--mask-open-iters", type=int, default=0, help="Mask opening iterations for noise cleanup")
    parser.add_argument("--mask-close-iters", type=int, default=0, help="Mask closing iterations for gap filling")
    parser.add_argument(
        "--min-area-ratio",
        type=float,
        default=0.003,
        help="Minimum contour area ratio against full frame",
    )
    parser.add_argument(
        "--auto-hold-seconds",
        type=float,
        default=2.0,
        help="How long to hold a V-sign before auto SVG flow starts",
    )
    parser.add_argument(
        "--pose-seconds",
        type=float,
        default=5.0,
        help="Time window after V-sign trigger to pose before auto SVG save",
    )
    return parser


def is_v_sign(hand_landmarks, img_h: int) -> bool:
    lm = hand_landmarks

    def y(idx: int) -> float:
        return lm[idx].y * img_h

    index_extended = y(8) < y(6)
    middle_extended = y(12) < y(10)
    ring_folded = y(16) > y(14)
    pinky_folded = y(20) > y(18)

    return index_extended and middle_extended and ring_folded and pinky_folded


def ensure_hand_landmarker_model(model_path: Path) -> None:
    if model_path.exists():
        return

    model_path.parent.mkdir(parents=True, exist_ok=True)
    model_url = (
        "https://storage.googleapis.com/mediapipe-models/"
        "hand_landmarker/hand_landmarker/float16/1/hand_landmarker.task"
    )
    urlretrieve(model_url, str(model_path))


def init_hand_tracker() -> tuple[object, str]:
    if hasattr(mp, "solutions"):
        mp_hands = mp.solutions.hands
        hands = mp_hands.Hands(
            static_image_mode=False,
            max_num_hands=2,
            min_detection_confidence=0.6,
            min_tracking_confidence=0.5,
        )
        return (
            {
                "backend": "solutions",
                "hands": hands,
                "mp_hands": mp_hands,
                "mp_draw": mp.solutions.drawing_utils,
            },
            "solutions",
        )

    from mediapipe.tasks import python as mp_tasks_python
    from mediapipe.tasks.python import vision

    model_path = Path("models/hand_landmarker.task")
    ensure_hand_landmarker_model(model_path)

    options = vision.HandLandmarkerOptions(
        base_options=mp_tasks_python.BaseOptions(model_asset_path=str(model_path)),
        running_mode=vision.RunningMode.IMAGE,
        num_hands=2,
        min_hand_detection_confidence=0.6,
        min_hand_presence_confidence=0.5,
        min_tracking_confidence=0.5,
    )
    hands = vision.HandLandmarker.create_from_options(options)
    return ({"backend": "tasks", "hands": hands}, "tasks")


def detect_v_sign(frame: np.ndarray, hand_tracker) -> tuple[bool, np.ndarray]:
    preview = frame.copy()
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    has_v_sign = False

    if hand_tracker["backend"] == "solutions":
        hands = hand_tracker["hands"]
        results = hands.process(rgb)
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                hand_tracker["mp_draw"].draw_landmarks(preview, hand_landmarks, hand_tracker["mp_hands"].HAND_CONNECTIONS)
                if is_v_sign(hand_landmarks.landmark, frame.shape[0]):
                    has_v_sign = True
        return has_v_sign, preview

    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
    results = hand_tracker["hands"].detect(mp_image)
    if results.hand_landmarks:
        h, w = frame.shape[:2]
        for hand_landmarks in results.hand_landmarks:
            for lm in hand_landmarks:
                px = int(lm.x * w)
                py = int(lm.y * h)
                if 0 <= px < w and 0 <= py < h:
                    cv2.circle(preview, (px, py), 2, (255, 200, 0), -1)
            if is_v_sign(hand_landmarks, h):
                has_v_sign = True

    return has_v_sign, preview


def draw_center_notice(frame: np.ndarray, text: str, color: tuple[int, int, int]) -> None:
    h, w = frame.shape[:2]
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 1.35
    thickness = 4
    (tw, th), _ = cv2.getTextSize(text, font, font_scale, thickness)

    x = max(20, (w - tw) // 2)
    y = h // 2
    pad = 18

    cv2.rectangle(
        frame,
        (x - pad, y - th - pad),
        (x + tw + pad, y + pad),
        (0, 0, 0),
        -1,
    )
    cv2.putText(frame, text, (x, y), font, font_scale, color, thickness, cv2.LINE_AA)


def draw_hud_panel(frame: np.ndarray, people_count: int) -> None:
    text = f"{people_count} person"
    x0, y0 = 18, 16
    panel_w = 220
    panel_h = 58

    overlay = frame.copy()
    cv2.rectangle(overlay, (x0, y0), (x0 + panel_w, y0 + panel_h), (12, 12, 12), -1)
    cv2.addWeighted(overlay, 0.65, frame, 0.35, 0, frame)
    cv2.rectangle(frame, (x0, y0), (x0 + panel_w, y0 + panel_h), (45, 210, 255), 2)
    cv2.putText(frame, text, (x0 + 14, y0 + 39), cv2.FONT_HERSHEY_SIMPLEX, 0.9, (80, 255, 120), 2, cv2.LINE_AA)


def draw_flash_countdown(frame: np.ndarray, seconds_left: float) -> None:
    if seconds_left > 5.0 or seconds_left <= 0:
        return

    value = int(np.ceil(seconds_left))
    value = max(1, min(5, value))
    progress = float(value) - float(seconds_left)
    show_digit = progress <= 0.58

    if not show_digit:
        return

    h, w = frame.shape[:2]
    text = str(value)
    scale = 8.0 - (progress * 2.0)
    scale = max(5.6, scale)
    thickness = 14
    (tw, th), _ = cv2.getTextSize(text, cv2.FONT_HERSHEY_DUPLEX, scale, thickness)
    x = (w - tw) // 2
    y = (h + th) // 2

    overlay = frame.copy()
    pad = 46
    cv2.rectangle(overlay, (x - pad, y - th - pad), (x + tw + pad, y + pad), (0, 0, 0), -1)
    cv2.addWeighted(overlay, 0.50, frame, 0.50, 0, frame)

    cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_DUPLEX, scale, (20, 20, 20), thickness + 8, cv2.LINE_AA)
    cv2.putText(frame, text, (x, y), cv2.FONT_HERSHEY_DUPLEX, scale, (40, 245, 255), thickness, cv2.LINE_AA)


def simplify_contour(contour: np.ndarray, epsilon_ratio: float) -> np.ndarray:
    if contour.shape[0] < 3 or epsilon_ratio <= 0:
        return contour

    contour_cv = contour.reshape((-1, 1, 2)).astype(np.int32)
    peri = cv2.arcLength(contour_cv, True)
    epsilon = peri * epsilon_ratio
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


def save_svg(contours: list[np.ndarray], width: int, height: int, output_path: Path) -> None:
    output_path.parent.mkdir(parents=True, exist_ok=True)
    polyline_lines = []
    for contour in contours:
        points = " ".join(f"{int(x)},{int(y)}" for x, y in contour)
        polyline_lines.append(
            f'  <polyline points="{points}" fill="none" stroke="black" stroke-width="2" '
            f'stroke-linejoin="round" stroke-linecap="round" />'
        )

    polyline_block = "\n".join(polyline_lines)
    svg_text = (
        f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}" '
        f'viewBox="0 0 {width} {height}">\n'
        f"{polyline_block}\n"
        "</svg>\n"
    )
    output_path.write_text(svg_text, encoding="utf-8")


def extract_person_contour_yolo(
    frame: np.ndarray,
    model,
    conf: float,
    imgsz: int,
    max_people: int,
    min_area_ratio: float,
    mask_open_iters: int,
    mask_close_iters: int,
) -> tuple[list[np.ndarray], list[tuple[int, int, int, int]], str]:
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
    max_keep = max_people if max_people > 0 else None

    if result.masks is not None and result.boxes is not None and len(result.boxes) > 0:
        masks = result.masks.data.cpu().numpy()
        boxes_xyxy = result.boxes.xyxy.cpu().numpy().astype(np.int32)
        scores = result.boxes.conf.cpu().numpy()

        found_items = []
        for idx, raw_mask in enumerate(masks):
            mask = (raw_mask > 0.5).astype(np.uint8) * 255
            if mask.shape[0] != frame_h or mask.shape[1] != frame_w:
                mask = cv2.resize(mask, (frame_w, frame_h), interpolation=cv2.INTER_NEAREST)

            if mask_open_iters > 0 or mask_close_iters > 0:
                kernel = np.ones((3, 3), np.uint8)
                if mask_open_iters > 0:
                    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=mask_open_iters)
                if mask_close_iters > 0:
                    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=mask_close_iters)

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
            contour_xy = contour.squeeze(axis=1)
            if contour_xy.ndim != 2 or contour_xy.shape[0] < 3:
                continue
            found_items.append((weight, contour_xy.astype(np.int32), bbox))

        if not found_items:
            return [], [], "YOLO found person but mask is too small"

        found_items.sort(key=lambda item: item[0], reverse=True)
        if max_keep is not None:
            found_items = found_items[:max_keep]

        contours = [item[1] for item in found_items]
        rois = [item[2] for item in found_items]
        return contours, rois, f"Segmented {len(contours)} person(s)"

    if result.boxes is not None and len(result.boxes) > 0:
        boxes_xyxy = result.boxes.xyxy.cpu().numpy().astype(np.int32)
        scores = result.boxes.conf.cpu().numpy()
        order = np.argsort(scores)[::-1]
        if max_keep is not None:
            order = order[:max_keep]

        contours = []
        rois = []
        for idx in order:
            x1, y1, x2, y2 = boxes_xyxy[int(idx)]
            bw = max(1, x2 - x1)
            bh = max(1, y2 - y1)
            bbox_area = float(bw * bh)
            if bbox_area < min_area:
                continue

            contour = np.array(
                [[x1, y1], [x2, y1], [x2, y2], [x1, y2]],
                dtype=np.int32,
            )
            contours.append(contour)
            rois.append((int(x1), int(y1), int(bw), int(bh)))

        if contours:
            return contours, rois, f"Using {len(contours)} YOLO bounding box(es)"
        return [], [], "YOLO box too small"

    return [], [], "No person detected"


def main() -> None:
    args = build_arg_parser().parse_args()

    if YOLO is None:
        raise RuntimeError("ultralytics is not installed. Run: pip install ultralytics")
    if mp is None:
        raise RuntimeError("mediapipe is not installed. Run: pip install mediapipe")

    model = YOLO(args.model)
    hand_tracker, tracker_backend = init_hand_tracker()

    cap = cv2.VideoCapture(args.camera)
    if not cap.isOpened():
        raise RuntimeError(f"Cannot open camera index {args.camera}.")

    cap.set(cv2.CAP_PROP_FRAME_WIDTH, args.width)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, args.height)

    output_path = Path(args.output)
    last_saved_path = ""
    last_status = "Waiting for detection"
    hold_start = None
    pose_deadline = None
    queued_contours = None
    queued_shape = None

    print("Camera started.")
    print("Press S to capture SVG, Q to quit.")
    print(f"Hold V-sign for {args.auto_hold_seconds:.1f}s, then pose for {args.pose_seconds:.1f}s.")
    print(f"Hand tracker backend: {tracker_backend}")
    print(f"YOLO model: {args.model}")

    try:
        while True:
            ok, frame = cap.read()
            if not ok:
                print("Failed to read frame from camera.")
                break

            contours, rois, status = extract_person_contour_yolo(
                frame,
                model,
                args.conf,
                args.imgsz,
                args.max_people,
                args.min_area_ratio,
                args.mask_open_iters,
                args.mask_close_iters,
            )
            last_status = status
            v_sign_detected, preview = detect_v_sign(frame, hand_tracker)
            now = time.time()
            gesture_text = ""

            if pose_deadline is not None:
                if contours:
                    queued_contours = [contour.copy() for contour in contours]
                    queued_shape = frame.shape[:2]

                left = pose_deadline - now
                if left > 0:
                    gesture_text = f"Pose now... SVG will be saved in {left:.1f}s"
                    draw_flash_countdown(preview, left)
                else:
                    if queued_contours is None or queued_shape is None:
                        print("Auto save blocked: no contour detected during pose window")
                        draw_center_notice(preview, "No contour found, save failed", (0, 80, 255))
                    else:
                        refined_contours = []
                        for contour in queued_contours:
                            refined = beautify_contour(contour, args.epsilon, args.smooth_iters, args.resample_step)
                            if refined.shape[0] >= 3:
                                refined_contours.append(refined)

                        if not refined_contours:
                            print("Auto save blocked: contour too small after simplify")
                            draw_center_notice(preview, "Contour too small, save failed", (0, 80, 255))
                        else:
                            timestamp = time.strftime("%Y%m%d_%H%M%S")
                            target = output_path.with_name(f"{output_path.stem}_{timestamp}.svg")
                            h, w = queued_shape
                            save_svg(refined_contours, w, h, target)
                            last_saved_path = str(target)
                            print(f"Auto SVG saved: {target}")
                            draw_center_notice(preview, "SVG auto-saved", (80, 255, 80))

                    pose_deadline = None
                    queued_contours = None
                    queued_shape = None
                    hold_start = None
            else:
                if v_sign_detected:
                    if hold_start is None:
                        hold_start = now
                    held_secs = now - hold_start
                    remaining = max(0.0, args.auto_hold_seconds - held_secs)
                    if held_secs >= args.auto_hold_seconds:
                        pose_deadline = now + args.pose_seconds
                        queued_contours = [contour.copy() for contour in contours] if contours else None
                        queued_shape = frame.shape[:2] if contours else None
                        hold_start = None
                        gesture_text = f"Triggered: start posing for {args.pose_seconds:.0f}s"
                        draw_center_notice(preview, gesture_text, (0, 255, 255))
                    else:
                        gesture_text = f"V-sign detected, hold for {remaining:.1f}s"
                        draw_center_notice(preview, gesture_text, (80, 255, 80))
                else:
                    hold_start = None
                    gesture_text = "Show a V-sign and hold for 2s"
                    draw_center_notice(preview, gesture_text, (80, 255, 80))

            for roi in rois:
                x, y, bw, bh = roi
                cv2.rectangle(preview, (x, y), (x + bw, y + bh), (255, 180, 0), 2)

            for contour in contours:
                cv2.polylines(preview, [contour.reshape((-1, 1, 2))], True, (0, 255, 0), 2)

            draw_hud_panel(preview, len(contours))

            cv2.imshow("YOLO Person Edge -> SVG", preview)

            key = cv2.waitKey(1) & 0xFF
            if key == ord("q"):
                break
            if key == ord("s"):
                if not contours:
                    print(f"Save blocked: {last_status}")
                    continue

                refined_contours = []
                for contour in contours:
                    refined = beautify_contour(contour, args.epsilon, args.smooth_iters, args.resample_step)
                    if refined.shape[0] >= 3:
                        refined_contours.append(refined)

                if not refined_contours:
                    print("Save blocked: contour too small after simplify")
                    continue

                timestamp = time.strftime("%Y%m%d_%H%M%S")
                target = output_path.with_name(f"{output_path.stem}_{timestamp}.svg")
                h, w = frame.shape[:2]
                save_svg(refined_contours, w, h, target)
                last_saved_path = str(target)
                print(f"Saved SVG: {target}")

    finally:
        hand_tracker["hands"].close()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
