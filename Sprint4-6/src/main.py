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


HAND_CONNECTIONS = (
    (0, 1), (1, 2), (2, 3), (3, 4),
    (0, 5), (5, 6), (6, 7), (7, 8),
    (5, 9), (9, 10), (10, 11), (11, 12),
    (9, 13), (13, 14), (14, 15), (15, 16),
    (13, 17), (17, 18), (18, 19), (19, 20),
    (0, 17),
)


def build_arg_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Use YOLO to detect person contours and export an SVG."
    )
    parser.add_argument(
        "--runtime-profile",
        type=str,
        choices=["quality", "balanced", "pi"],
        default="balanced",
        help="Runtime preset. Use 'pi' for Raspberry Pi deployment.",
    )
    parser.add_argument("--camera", type=int, default=0, help="Camera index")
    parser.add_argument("--width", type=int, default=1280, help="Capture width")
    parser.add_argument("--height", type=int, default=720, help="Capture height")
    parser.add_argument("--output", type=str, default="output/person_edge.svg", help="Output SVG path")
    parser.add_argument("--epsilon", type=float, default=0.0015, help="Contour simplify ratio (0 keeps raw contour)")
    parser.add_argument("--model", type=str, default="yolov8n-seg.pt", help="YOLO segmentation model path")
    parser.add_argument("--conf", type=float, default=0.25, help="YOLO confidence threshold")
    parser.add_argument("--imgsz", type=int, default=960, help="YOLO inference size")
    parser.add_argument(
        "--mask-threshold",
        type=float,
        default=0.35,
        help="Mask binarization threshold (lower keeps thinner details)",
    )
    parser.add_argument(
        "--retina-masks",
        dest="retina_masks",
        action="store_true",
        help="Use high-resolution segmentation masks for finer contour edges",
    )
    parser.add_argument(
        "--no-retina-masks",
        dest="retina_masks",
        action="store_false",
        help="Disable high-resolution masks to improve speed",
    )
    parser.set_defaults(retina_masks=True)
    parser.add_argument(
        "--max-people",
        type=int,
        default=1,
        help="Reserved; current pipeline outputs a single full-frame contour",
    )
    parser.add_argument(
        "--detect-interval",
        type=int,
        default=1,
        help="Run YOLO every N frames and reuse previous result in between",
    )
    parser.add_argument(
        "--process-scale",
        type=float,
        default=1.0,
        help="Scale factor for YOLO input frame (0.3~1.0)",
    )
    parser.add_argument(
        "--hand-refine",
        dest="hand_refine",
        action="store_true",
        help="Refine person mask around hand landmarks for clearer finger contours",
    )
    parser.add_argument(
        "--no-hand-refine",
        dest="hand_refine",
        action="store_false",
        help="Disable hand landmark refinement",
    )
    parser.add_argument("--hand-edge-low", type=int, default=40, help="Canny low threshold for hand detail enhancement")
    parser.add_argument("--hand-edge-high", type=int, default=120, help="Canny high threshold for hand detail enhancement")
    parser.set_defaults(hand_refine=True)
    parser.add_argument(
        "--show-hand-landmarks",
        dest="show_hand_landmarks",
        action="store_true",
        help="Draw hand landmarks on preview (disabled by default to avoid visual overlap)",
    )
    parser.set_defaults(show_hand_landmarks=False)
    parser.add_argument(
        "--hand-local-detail",
        dest="hand_local_detail",
        action="store_true",
        help="Use dedicated mask threshold in hand regions to preserve finger gaps",
    )
    parser.add_argument(
        "--no-hand-local-detail",
        dest="hand_local_detail",
        action="store_false",
        help="Disable hand-region local detail mask",
    )
    parser.add_argument(
        "--hand-mask-threshold",
        type=float,
        default=0.5,
        help="Mask threshold used only in hand regions (higher helps separate fingers)",
    )
    parser.add_argument(
        "--hand-mask-low-threshold",
        type=float,
        default=0.2,
        help="Low threshold used as fallback to recover outstretched hands",
    )
    parser.add_argument(
        "--hand-region-expand",
        type=float,
        default=0.1,
        help="Extra expansion ratio for hand local-detail region",
    )
    parser.add_argument(
        "--hand-component-min-area",
        type=int,
        default=18,
        help="Minimum connected component size kept in hand region",
    )
    parser.add_argument(
        "--hand-grabcut",
        dest="hand_grabcut",
        action="store_true",
        help="Run GrabCut refinement in hand ROIs for higher hand precision",
    )
    parser.add_argument(
        "--no-hand-grabcut",
        dest="hand_grabcut",
        action="store_false",
        help="Disable GrabCut hand refinement",
    )
    parser.add_argument(
        "--hand-grabcut-iters",
        type=int,
        default=2,
        help="GrabCut iterations for each hand ROI",
    )
    parser.add_argument(
        "--hand-grabcut-pad",
        type=float,
        default=0.35,
        help="Extra ROI padding ratio around each hand for GrabCut",
    )
    parser.add_argument(
        "--hand-grabcut-min-size",
        type=int,
        default=36,
        help="Minimum hand ROI width/height for GrabCut",
    )
    parser.set_defaults(hand_local_detail=True)
    parser.set_defaults(hand_grabcut=True)
    parser.add_argument("--smooth-iters", type=int, default=1, help="Chaikin smoothing iterations")
    parser.add_argument("--resample-step", type=float, default=2.0, help="Target pixel step for SVG points (0 disables resampling)")
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


def apply_runtime_profile(args) -> None:
    if args.runtime_profile == "quality":
        return

    if args.runtime_profile == "balanced":
        args.detect_interval = max(1, args.detect_interval)
        args.process_scale = float(np.clip(args.process_scale, 0.5, 1.0))
        return

    # Raspberry Pi preset: prioritize responsiveness and deployment stability.
    args.imgsz = min(args.imgsz, 512)
    args.retina_masks = False
    args.hand_refine = False
    args.hand_local_detail = False
    args.hand_grabcut = False
    args.detect_interval = max(args.detect_interval, 3)
    args.process_scale = float(np.clip(min(args.process_scale, 0.6), 0.35, 1.0))
    args.smooth_iters = min(args.smooth_iters, 1)
    if args.resample_step <= 0:
        args.resample_step = 3.0


def scale_hand_landmarks(hand_sets: list[np.ndarray], scale: float) -> list[np.ndarray]:
    if abs(scale - 1.0) < 1e-6:
        return hand_sets

    scaled = []
    for pts in hand_sets:
        p = np.rint(pts.astype(np.float32) * scale).astype(np.int32)
        scaled.append(p)
    return scaled


def scale_contours_and_rois(
    contours: list[np.ndarray],
    rois: list[tuple[int, int, int, int]],
    inv_scale: float,
    frame_w: int,
    frame_h: int,
) -> tuple[list[np.ndarray], list[tuple[int, int, int, int]]]:
    if abs(inv_scale - 1.0) < 1e-6:
        return contours, rois

    scaled_contours = []
    for contour in contours:
        pts = np.rint(contour.astype(np.float32) * inv_scale).astype(np.int32)
        pts[:, 0] = np.clip(pts[:, 0], 0, frame_w - 1)
        pts[:, 1] = np.clip(pts[:, 1], 0, frame_h - 1)
        scaled_contours.append(pts)

    scaled_rois = []
    for x, y, w, h in rois:
        sx = int(np.clip(round(x * inv_scale), 0, frame_w - 1))
        sy = int(np.clip(round(y * inv_scale), 0, frame_h - 1))
        sw = int(max(1, round(w * inv_scale)))
        sh = int(max(1, round(h * inv_scale)))
        sw = min(sw, frame_w - sx)
        sh = min(sh, frame_h - sy)
        scaled_rois.append((sx, sy, max(1, sw), max(1, sh)))

    return scaled_contours, scaled_rois


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


def detect_v_sign(frame: np.ndarray, hand_tracker, show_hand_landmarks: bool) -> tuple[bool, np.ndarray, list[np.ndarray]]:
    preview = frame.copy()
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    has_v_sign = False
    hand_landmarks_px: list[np.ndarray] = []
    h, w = frame.shape[:2]

    if hand_tracker["backend"] == "solutions":
        hands = hand_tracker["hands"]
        results = hands.process(rgb)
        if results.multi_hand_landmarks:
            for hand_landmarks in results.multi_hand_landmarks:
                if show_hand_landmarks:
                    hand_tracker["mp_draw"].draw_landmarks(preview, hand_landmarks, hand_tracker["mp_hands"].HAND_CONNECTIONS)
                pts = []
                for lm in hand_landmarks.landmark:
                    px = int(np.clip(lm.x * w, 0, w - 1))
                    py = int(np.clip(lm.y * h, 0, h - 1))
                    pts.append([px, py])
                if pts:
                    hand_landmarks_px.append(np.asarray(pts, dtype=np.int32))
                if is_v_sign(hand_landmarks.landmark, frame.shape[0]):
                    has_v_sign = True
        return has_v_sign, preview, hand_landmarks_px

    mp_image = mp.Image(image_format=mp.ImageFormat.SRGB, data=rgb)
    results = hand_tracker["hands"].detect(mp_image)
    if results.hand_landmarks:
        for hand_landmarks in results.hand_landmarks:
            pts = []
            for lm in hand_landmarks:
                px = int(np.clip(lm.x * w, 0, w - 1))
                py = int(np.clip(lm.y * h, 0, h - 1))
                pts.append([px, py])
                if show_hand_landmarks and 0 <= px < w and 0 <= py < h:
                    cv2.circle(preview, (px, py), 2, (255, 200, 0), -1)
            if pts:
                hand_landmarks_px.append(np.asarray(pts, dtype=np.int32))
            if is_v_sign(hand_landmarks, h):
                has_v_sign = True

    return has_v_sign, preview, hand_landmarks_px


def select_hands_for_bbox(
    hand_landmarks_px: list[np.ndarray],
    bbox_xyxy: tuple[int, int, int, int],
    frame_w: int,
    frame_h: int,
) -> list[np.ndarray]:
    x1, y1, x2, y2 = bbox_xyxy
    margin_x = int(max(18, (x2 - x1) * 0.25))
    margin_y = int(max(18, (y2 - y1) * 0.25))
    ex1 = max(0, x1 - margin_x)
    ey1 = max(0, y1 - margin_y)
    ex2 = min(frame_w - 1, x2 + margin_x)
    ey2 = min(frame_h - 1, y2 + margin_y)

    selected = []
    for pts in hand_landmarks_px:
        if pts.size == 0:
            continue
        cx = int(np.mean(pts[:, 0]))
        cy = int(np.mean(pts[:, 1]))
        if ex1 <= cx <= ex2 and ey1 <= cy <= ey2:
            selected.append(pts)
    return selected


def build_hand_region_mask(shape_hw: tuple[int, int], hand_sets: list[np.ndarray], expand_ratio: float) -> np.ndarray:
    region = np.zeros(shape_hw, dtype=np.uint8)
    kernel = np.ones((3, 3), np.uint8)
    expand = max(0.0, float(expand_ratio))

    for pts in hand_sets:
        if pts.shape[0] < 3:
            continue
        hull = cv2.convexHull(pts)
        cv2.fillConvexPoly(region, hull, 255)
        span = float(max(np.ptp(pts[:, 0]), np.ptp(pts[:, 1])))
        pad = max(2, int(span * expand))
        for x, y in pts:
            cv2.circle(region, (int(x), int(y)), pad, 255, -1)

    region = cv2.morphologyEx(region, cv2.MORPH_CLOSE, kernel, iterations=1)
    return region


def build_hand_anchor_mask(shape_hw: tuple[int, int], hand_sets: list[np.ndarray]) -> np.ndarray:
    anchors = np.zeros(shape_hw, dtype=np.uint8)

    for pts in hand_sets:
        if pts.shape[0] < 3:
            continue

        span = float(max(np.ptp(pts[:, 0]), np.ptp(pts[:, 1])))
        r = max(2, int(span * 0.03))
        for x, y in pts:
            cv2.circle(anchors, (int(x), int(y)), r, 255, -1)

    return anchors


def keep_components_touching_anchors(binary_mask: np.ndarray, anchor_mask: np.ndarray, min_area: int) -> np.ndarray:
    if not np.any(binary_mask):
        return binary_mask

    n, labels, stats, _ = cv2.connectedComponentsWithStats((binary_mask > 0).astype(np.uint8), connectivity=8)
    out = np.zeros_like(binary_mask)
    anchor_labels = set(np.unique(labels[anchor_mask > 0]).tolist())

    for label in range(1, n):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if area < min_area:
            continue
        if label not in anchor_labels:
            continue
        out[labels == label] = 255

    return out


def estimate_hand_component_min_area(hand_sets: list[np.ndarray], floor_area: int) -> int:
    areas = []
    for pts in hand_sets:
        if pts.shape[0] < 3:
            continue
        span = float(max(np.ptp(pts[:, 0]), np.ptp(pts[:, 1])))
        areas.append(max(4, int((span * span) * 0.0025)))

    est = max(areas) if areas else 0
    return max(int(floor_area), int(est))


def build_hand_detail_mask(
    shape_hw: tuple[int, int],
    hand_sets: list[np.ndarray],
    edge_map: np.ndarray | None,
) -> np.ndarray:
    detail_mask = np.zeros(shape_hw, dtype=np.uint8)
    kernel = np.ones((3, 3), np.uint8)

    for pts in hand_sets:
        if pts.shape[0] < 5:
            continue

        span = float(max(np.ptp(pts[:, 0]), np.ptp(pts[:, 1])))
        thickness = max(1, int(span * 0.03))
        guide = np.zeros(shape_hw, dtype=np.uint8)

        for i0, i1 in HAND_CONNECTIONS:
            if i0 < pts.shape[0] and i1 < pts.shape[0]:
                cv2.line(guide, tuple(pts[i0]), tuple(pts[i1]), 255, thickness=thickness)

        for x, y in pts:
            cv2.circle(guide, (int(x), int(y)), max(1, thickness // 2), 255, -1)

        guide = cv2.dilate(guide, kernel, iterations=1)

        detail_mask = cv2.bitwise_or(detail_mask, guide)
        if edge_map is not None:
            hand_band = cv2.dilate(guide, kernel, iterations=2)
            hand_edges = cv2.bitwise_and(edge_map, hand_band)
            hand_edges = cv2.morphologyEx(hand_edges, cv2.MORPH_CLOSE, kernel, iterations=1)
            hand_edges = cv2.dilate(hand_edges, kernel, iterations=1)
            detail_mask = cv2.bitwise_or(detail_mask, hand_edges)

    return detail_mask


def apply_hand_local_detail(
    mask_prob: np.ndarray,
    base_mask: np.ndarray,
    hand_sets: list[np.ndarray],
    hand_mask_threshold: float,
    hand_mask_low_threshold: float,
    hand_region_expand: float,
    hand_component_min_area: int,
) -> np.ndarray:
    if not hand_sets:
        return base_mask

    hand_region = build_hand_region_mask(base_mask.shape, hand_sets, hand_region_expand)
    if not np.any(hand_region):
        return base_mask

    hand_probs = mask_prob[hand_region > 0]
    adaptive = float(np.quantile(hand_probs, 0.58)) if hand_probs.size > 0 else hand_mask_threshold
    core_thresh = float(np.clip((hand_mask_threshold + adaptive) * 0.5, 0.08, 0.95))
    low_thresh = float(np.clip(min(hand_mask_low_threshold, core_thresh - 0.02), 0.03, 0.9))

    hand_core = (mask_prob >= core_thresh).astype(np.uint8) * 255
    hand_support = (mask_prob >= low_thresh).astype(np.uint8) * 255
    hand_core = cv2.bitwise_and(hand_core, hand_region)
    hand_support = cv2.bitwise_and(hand_support, hand_region)

    anchors = build_hand_anchor_mask(base_mask.shape, hand_sets)
    min_area = estimate_hand_component_min_area(hand_sets, hand_component_min_area)

    core_keep = keep_components_touching_anchors(hand_core, anchors, min_area)
    if np.any(core_keep):
        seed = cv2.bitwise_or(anchors, core_keep)
        seed = cv2.dilate(seed, np.ones((3, 3), np.uint8), iterations=1)
        support_keep = keep_components_touching_anchors(hand_support, seed, min_area)
        hand_mask = cv2.bitwise_or(core_keep, support_keep)
    else:
        hand_mask = keep_components_touching_anchors(hand_support, anchors, min_area)

    merged = base_mask.copy()
    hand_idx = hand_region > 0
    merged[hand_idx] = hand_mask[hand_idx]
    return merged


def carve_finger_gaps(mask: np.ndarray, hand_sets: list[np.ndarray]) -> np.ndarray:
    carved = mask.copy()
    for pts in hand_sets:
        if pts.shape[0] < 21:
            continue

        span = float(max(np.ptp(pts[:, 0]), np.ptp(pts[:, 1])))
        thickness = max(1, int(span * 0.02))
        palm = pts[0].astype(np.int32)

        gap_mask = np.zeros_like(mask)
        hand_region = np.zeros_like(mask)
        cv2.fillConvexPoly(hand_region, cv2.convexHull(pts), 255)

        # Cut thin paths at finger valleys to keep visible finger separations.
        gap_pairs = ((8, 12, 6, 10), (12, 16, 10, 14), (16, 20, 14, 18))
        for t0, t1, b0, b1 in gap_pairs:
            tip_mid = np.rint((pts[t0] + pts[t1]) * 0.5).astype(np.int32)
            base_mid = np.rint((pts[b0] + pts[b1]) * 0.5).astype(np.int32)
            end = np.rint(base_mid * 0.75 + palm * 0.25).astype(np.int32)
            cv2.line(gap_mask, tuple(tip_mid), tuple(end), 255, thickness=thickness)

        gap_mask = cv2.bitwise_and(gap_mask, hand_region)
        carved[gap_mask > 0] = 0

    return carved


def refine_mask_with_hands(
    mask: np.ndarray,
    hand_sets: list[np.ndarray],
    edge_map: np.ndarray | None,
    hand_region_expand: float,
    hand_component_min_area: int,
) -> np.ndarray:
    if not hand_sets:
        return mask

    merged = carve_finger_gaps(mask, hand_sets)
    hand_region = build_hand_region_mask(mask.shape, hand_sets, hand_region_expand + 0.04)
    anchors = build_hand_anchor_mask(mask.shape, hand_sets)
    min_area = estimate_hand_component_min_area(hand_sets, hand_component_min_area)

    inside = cv2.bitwise_and(merged, hand_region)
    inside = keep_components_touching_anchors(inside, anchors, min_area)

    if edge_map is not None and np.any(inside):
        kernel = np.ones((3, 3), np.uint8)
        protected = cv2.dilate(anchors, kernel, iterations=2)
        edge_cut = cv2.bitwise_and(edge_map, hand_region)
        edge_cut = cv2.morphologyEx(edge_cut, cv2.MORPH_OPEN, kernel, iterations=1)
        edge_cut = cv2.bitwise_and(edge_cut, cv2.bitwise_not(protected))
        inside[edge_cut > 0] = 0

    outside = cv2.bitwise_and(merged, cv2.bitwise_not(hand_region))
    merged = cv2.bitwise_or(outside, inside)
    merged = cv2.morphologyEx(merged, cv2.MORPH_OPEN, np.ones((3, 3), np.uint8), iterations=1)
    return merged


def refine_hands_with_grabcut(
    frame: np.ndarray,
    mask: np.ndarray,
    hand_sets: list[np.ndarray],
    hand_grabcut_pad: float,
    hand_grabcut_iters: int,
    hand_grabcut_min_size: int,
    hand_component_min_area: int,
) -> np.ndarray:
    if not hand_sets or hand_grabcut_iters <= 0:
        return mask

    out = mask.copy()
    frame_h, frame_w = mask.shape[:2]
    grab_iters = int(np.clip(hand_grabcut_iters, 1, 5))
    pad_ratio = max(0.0, float(hand_grabcut_pad))
    min_side = max(12, int(hand_grabcut_min_size))

    for pts in hand_sets:
        if pts.shape[0] < 5:
            continue

        min_x = int(np.min(pts[:, 0]))
        max_x = int(np.max(pts[:, 0]))
        min_y = int(np.min(pts[:, 1]))
        max_y = int(np.max(pts[:, 1]))
        span = float(max(max_x - min_x, max_y - min_y))
        pad = max(6, int(span * pad_ratio))

        x1 = max(0, min_x - pad)
        y1 = max(0, min_y - pad)
        x2 = min(frame_w - 1, max_x + pad)
        y2 = min(frame_h - 1, max_y + pad)

        if (x2 - x1 + 1) < min_side or (y2 - y1 + 1) < min_side:
            continue

        roi_img = frame[y1:y2 + 1, x1:x2 + 1]
        roi_mask = out[y1:y2 + 1, x1:x2 + 1]
        local_pts = pts.copy()
        local_pts[:, 0] -= x1
        local_pts[:, 1] -= y1

        gc_mask = np.full(roi_mask.shape, cv2.GC_BGD, dtype=np.uint8)
        gc_mask[roi_mask > 0] = cv2.GC_PR_FGD

        border = max(2, int(min(roi_mask.shape[0], roi_mask.shape[1]) * 0.04))
        gc_mask[:border, :] = cv2.GC_BGD
        gc_mask[-border:, :] = cv2.GC_BGD
        gc_mask[:, :border] = cv2.GC_BGD
        gc_mask[:, -border:] = cv2.GC_BGD

        anchor_r = max(2, int(span * 0.03))
        for x, y in local_pts:
            cv2.circle(gc_mask, (int(x), int(y)), anchor_r, cv2.GC_FGD, -1)

        bgd_model = np.zeros((1, 65), np.float64)
        fgd_model = np.zeros((1, 65), np.float64)
        try:
            cv2.grabCut(roi_img, gc_mask, None, bgd_model, fgd_model, grab_iters, cv2.GC_INIT_WITH_MASK)
        except cv2.error:
            continue

        fg = np.where((gc_mask == cv2.GC_FGD) | (gc_mask == cv2.GC_PR_FGD), 255, 0).astype(np.uint8)

        region = np.zeros_like(fg)
        if local_pts.shape[0] >= 3:
            cv2.fillConvexPoly(region, cv2.convexHull(local_pts.astype(np.int32)), 255)
        for x, y in local_pts:
            cv2.circle(region, (int(x), int(y)), max(2, int(span * 0.08)), 255, -1)
        region = cv2.dilate(region, np.ones((3, 3), np.uint8), iterations=2)

        fg = cv2.bitwise_and(fg, region)
        anchor_mask = np.zeros_like(fg)
        for x, y in local_pts:
            cv2.circle(anchor_mask, (int(x), int(y)), anchor_r, 255, -1)

        min_area = estimate_hand_component_min_area([pts], hand_component_min_area)
        fg = keep_components_touching_anchors(fg, anchor_mask, min_area)
        if not np.any(fg):
            continue

        roi_out = out[y1:y2 + 1, x1:x2 + 1]
        replace_idx = region > 0
        roi_out[replace_idx] = fg[replace_idx]

    return out


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


def draw_top_notice(frame: np.ndarray, text: str, color: tuple[int, int, int]) -> None:
    h, w = frame.shape[:2]
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = 0.95
    thickness = 2
    (tw, th), _ = cv2.getTextSize(text, font, font_scale, thickness)

    x = max(16, (w - tw) // 2)
    y = max(30, int(h * 0.10))
    pad_x = 14
    pad_y = 10

    overlay = frame.copy()
    cv2.rectangle(
        overlay,
        (x - pad_x, y - th - pad_y),
        (x + tw + pad_x, y + pad_y),
        (0, 0, 0),
        -1,
    )
    cv2.addWeighted(overlay, 0.55, frame, 0.45, 0, frame)
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


def smooth_contour_points(contour: np.ndarray, window: int = 5, iterations: int = 1) -> np.ndarray:
    if contour.shape[0] < max(7, window + 2):
        return contour

    if window % 2 == 0:
        window += 1
    window = max(3, window)
    iterations = max(1, iterations)

    pts = contour.astype(np.float32)
    kernel = np.ones(window, dtype=np.float32) / float(window)
    pad = window // 2

    for _ in range(iterations):
        ext = np.vstack([pts[-pad:], pts, pts[:pad]])
        xs = np.convolve(ext[:, 0], kernel, mode="valid")
        ys = np.convolve(ext[:, 1], kernel, mode="valid")
        pts = np.stack([xs, ys], axis=1)

    return np.rint(pts).astype(np.int32)


def keep_main_and_hand_components(
    mask: np.ndarray,
    min_main_area: int,
    hand_anchor_mask: np.ndarray,
    min_hand_area: int,
) -> np.ndarray:
    if not np.any(mask):
        return mask

    binary = (mask > 0).astype(np.uint8)
    count, labels, stats, _ = cv2.connectedComponentsWithStats(binary, connectivity=8)
    if count <= 1:
        return mask

    best_label = 0
    best_area = 0
    for label in range(1, count):
        area = int(stats[label, cv2.CC_STAT_AREA])
        if area > best_area:
            best_area = area
            best_label = label

    keep_labels = set()
    if best_label != 0 and best_area >= int(min_main_area):
        keep_labels.add(best_label)

    if hand_anchor_mask is not None and np.any(hand_anchor_mask):
        anchor_labels = np.unique(labels[hand_anchor_mask > 0])
        for label in anchor_labels:
            if label <= 0:
                continue
            area = int(stats[int(label), cv2.CC_STAT_AREA])
            if area >= int(min_hand_area):
                keep_labels.add(int(label))

    if not keep_labels:
        return np.zeros_like(mask)

    out = np.zeros_like(mask)
    for label in keep_labels:
        out[labels == label] = 255
    return out


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
    mask_threshold: float,
    retina_masks: bool,
    hand_landmarks_px: list[np.ndarray],
    hand_refine: bool,
    hand_edge_low: int,
    hand_edge_high: int,
    hand_local_detail: bool,
    hand_mask_threshold: float,
    hand_mask_low_threshold: float,
    hand_region_expand: float,
    hand_component_min_area: int,
    hand_grabcut: bool,
    hand_grabcut_pad: float,
    hand_grabcut_iters: int,
    hand_grabcut_min_size: int,
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
        retina_masks=retina_masks,
        verbose=False,
        device="cpu",
    )[0]

    frame_h, frame_w = frame.shape[:2]
    min_area = float(frame_h * frame_w) * min_area_ratio
    edge_map = None
    if hand_refine and hand_landmarks_px:
        low = int(np.clip(hand_edge_low, 1, 254))
        high = int(np.clip(hand_edge_high, low + 1, 255))
        edge_map = cv2.Canny(cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY), low, high)

    if result.masks is not None and result.boxes is not None and len(result.boxes) > 0:
        masks = result.masks.data.cpu().numpy()
        resized_probs = []
        for raw_mask in masks:
            mask_prob = raw_mask.astype(np.float32)
            if mask_prob.shape[0] != frame_h or mask_prob.shape[1] != frame_w:
                mask_prob = cv2.resize(mask_prob, (frame_w, frame_h), interpolation=cv2.INTER_LINEAR)
            resized_probs.append(mask_prob)

        if not resized_probs:
            return [], [], "YOLO found person but mask is too small"

        merged_prob = np.max(np.stack(resized_probs, axis=0), axis=0)
        thresh = float(np.clip(mask_threshold, 0.05, 0.95))
        mask = (merged_prob >= thresh).astype(np.uint8) * 255

        if mask_open_iters > 0 or mask_close_iters > 0:
            kernel = np.ones((3, 3), np.uint8)
            if mask_open_iters > 0:
                mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel, iterations=mask_open_iters)
            if mask_close_iters > 0:
                mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=mask_close_iters)

        hand_sets = hand_landmarks_px if hand_landmarks_px else []
        if hand_local_detail and hand_sets:
            mask = apply_hand_local_detail(
                merged_prob,
                mask,
                hand_sets,
                hand_mask_threshold,
                hand_mask_low_threshold,
                hand_region_expand,
                hand_component_min_area,
            )

        if hand_refine and hand_sets:
            mask = refine_mask_with_hands(mask, hand_sets, edge_map, hand_region_expand, hand_component_min_area)

        if hand_grabcut and hand_sets:
            mask = refine_hands_with_grabcut(
                frame,
                mask,
                hand_sets,
                hand_grabcut_pad,
                hand_grabcut_iters,
                hand_grabcut_min_size,
                hand_component_min_area,
            )

        # Full-frame mode: keep dominant body component plus hand-anchor-connected components.
        anchor_mask = build_hand_anchor_mask(mask.shape, hand_sets) if hand_sets else np.zeros_like(mask)
        hand_keep_area = estimate_hand_component_min_area(hand_sets, hand_component_min_area) if hand_sets else hand_component_min_area
        mask = keep_main_and_hand_components(mask, int(min_area), anchor_mask, int(hand_keep_area))
        if not np.any(mask):
            return [], [], "No dominant person region after filtering"

        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)
        if not contours:
            return [], [], "YOLO found person but mask is too small"

        contour = max(contours, key=cv2.contourArea)
        area = float(cv2.contourArea(contour))
        if area < min_area:
            return [], [], "YOLO found person but mask is too small"

        contour_xy = contour.squeeze(axis=1)
        if contour_xy.ndim != 2 or contour_xy.shape[0] < 3:
            return [], [], "Invalid contour from full-frame segmentation"

        contour_xy = smooth_contour_points(contour_xy.astype(np.int32), window=5, iterations=1)
        x, y, w, h = cv2.boundingRect(contour_xy.reshape((-1, 1, 2)))
        bbox = (int(x), int(y), int(max(1, w)), int(max(1, h)))
        return [contour_xy], [bbox], "Segmented full-frame single contour"

    if result.boxes is not None and len(result.boxes) > 0:
        boxes_xyxy = result.boxes.xyxy.cpu().numpy().astype(np.int32)
        scores = result.boxes.conf.cpu().numpy()
        best_idx = int(np.argmax(scores))
        x1, y1, x2, y2 = boxes_xyxy[best_idx]
        bw = max(1, x2 - x1)
        bh = max(1, y2 - y1)
        bbox_area = float(bw * bh)
        if bbox_area >= min_area:
            contour = np.array(
                [[x1, y1], [x2, y1], [x2, y2], [x1, y2]],
                dtype=np.int32,
            )
            return [contour], [(int(x1), int(y1), int(bw), int(bh))], "Using single YOLO bounding box"
        return [], [], "YOLO box too small"

    return [], [], "No person detected"


def main() -> None:
    args = build_arg_parser().parse_args()
    apply_runtime_profile(args)

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
    success_notice_until = 0.0
    detect_interval = max(1, int(args.detect_interval))
    process_scale = float(np.clip(args.process_scale, 0.3, 1.0))
    frame_idx = 0
    cached_contours: list[np.ndarray] = []
    cached_rois: list[tuple[int, int, int, int]] = []
    cached_status = "Waiting for detection"

    print("Camera started.")
    print("Press S to capture SVG, Q to quit.")
    print(f"Hold V-sign for {args.auto_hold_seconds:.1f}s, then pose for {args.pose_seconds:.1f}s.")
    print(f"Hand tracker backend: {tracker_backend}")
    print(f"YOLO model: {args.model}")
    print(f"Runtime profile: {args.runtime_profile}")
    print(f"Detect interval: {detect_interval} frame(s), process scale: {process_scale:.2f}")

    try:
        while True:
            frame_idx += 1
            ok, frame = cap.read()
            if not ok:
                print("Failed to read frame from camera.")
                break

            v_sign_detected, preview, hand_landmarks_px = detect_v_sign(frame, hand_tracker, args.show_hand_landmarks)
            need_detect = (frame_idx % detect_interval == 0) or not cached_contours

            if need_detect:
                infer_frame = frame
                infer_hands = hand_landmarks_px
                inv_scale = 1.0

                if process_scale < 0.999:
                    infer_frame = cv2.resize(
                        frame,
                        (int(frame.shape[1] * process_scale), int(frame.shape[0] * process_scale)),
                        interpolation=cv2.INTER_LINEAR,
                    )
                    infer_hands = scale_hand_landmarks(hand_landmarks_px, process_scale)
                    inv_scale = 1.0 / process_scale

                contours, rois, status = extract_person_contour_yolo(
                    infer_frame,
                    model,
                    args.conf,
                    args.imgsz,
                    args.mask_threshold,
                    args.retina_masks,
                    infer_hands,
                    args.hand_refine,
                    args.hand_edge_low,
                    args.hand_edge_high,
                    args.hand_local_detail,
                    args.hand_mask_threshold,
                    args.hand_mask_low_threshold,
                    args.hand_region_expand,
                    args.hand_component_min_area,
                    args.hand_grabcut,
                    args.hand_grabcut_pad,
                    args.hand_grabcut_iters,
                    args.hand_grabcut_min_size,
                    args.max_people,
                    args.min_area_ratio,
                    args.mask_open_iters,
                    args.mask_close_iters,
                )

                if process_scale < 0.999:
                    contours, rois = scale_contours_and_rois(
                        contours,
                        rois,
                        inv_scale,
                        frame.shape[1],
                        frame.shape[0],
                    )

                cached_contours = [c.copy() for c in contours]
                cached_rois = list(rois)
                cached_status = status

            contours = [c.copy() for c in cached_contours]
            rois = list(cached_rois)
            status = cached_status
            last_status = status
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
                            success_notice_until = time.time() + 1.5

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
                    draw_top_notice(preview, gesture_text, (80, 255, 80))

            for roi in rois:
                x, y, bw, bh = roi
                cv2.rectangle(preview, (x, y), (x + bw, y + bh), (255, 180, 0), 2)

            for contour in contours:
                cv2.polylines(preview, [contour.reshape((-1, 1, 2))], True, (0, 255, 0), 2)

            draw_hud_panel(preview, len(contours))
            if now < success_notice_until:
                draw_center_notice(preview, "Capture successful", (80, 255, 80))

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
                success_notice_until = time.time() + 1.5

    finally:
        hand_tracker["hands"].close()
        cap.release()
        cv2.destroyAllWindows()


if __name__ == "__main__":
    main()
