import argparse
import time
from pathlib import Path
import cv2
import numpy as np
import subprocess
import os

try:
    from ultralytics import YOLO
except ImportError:
    print("错误：未找到 ultralytics。")
    exit(1)

def build_arg_parser():
    parser = argparse.ArgumentParser(description="YOLO Restore Vision")
    parser.add_argument("--camera", type=int, default=0)
    parser.add_argument("--width", type=int, default=1280)
    parser.add_argument("--height", type=int, default=720)
    parser.add_argument("--output", type=str, default="output/pi_draw.svg")
    parser.add_argument("--model", type=str, default="yolov8n-seg.pt")
    parser.add_argument("--conf", type=float, default=0.25) # 降低默认阈值提高灵敏度
    parser.add_argument("--auto-trigger-seconds", type=float, default=2.0)
    parser.add_argument("--pose-seconds", type=float, default=5.0)
    return parser

def save_svg(contours, width, height, output_path):
    output_path.parent.mkdir(parents=True, exist_ok=True)
    lines = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{width}" height="{height}">' ]
    for c in contours:
        pts = " ".join(f"{int(x)},{int(y)}" for x, y in c)
        lines.append(f'  <polyline points="{pts}" fill="none" stroke="black" stroke-width="2"/>')
    lines.append("</svg>")
    output_path.write_text("\n".join(lines))

def simplify_contour(contour, epsilon_ratio):
    if contour.shape[0] < 3 or epsilon_ratio <= 0: return contour
    peri = cv2.arcLength(contour, True)
    return cv2.approxPolyDP(contour, peri * epsilon_ratio, True).reshape((-1, 2))

def main():
    args = build_arg_parser().parse_args()
    model = YOLO(args.model)
    cap = cv2.VideoCapture(args.camera)
    cap.set(3, args.width)
    cap.set(4, args.height)

    # --- 还原为普通窗口，解决拉伸问题 ---
    win_name = "AI Art System"
    cv2.namedWindow(win_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win_name, 1280, 720)

    person_seen_start = None
    pose_deadline = None
    
    print("视觉增强版已就绪。正在运行...")

    while True:
        ok, frame = cap.read()
        if not ok: break
        
        # 移除了严格的 classes=[0] 过滤，增加容错
        results = model.predict(source=frame, conf=args.conf, verbose=False)[0]
        preview = frame.copy()
        now = time.time()
        
        # 兼容性检测：只要有框或者有掩码都算检测到
        has_detection = len(results.boxes) > 0
        
        if pose_deadline:
            left = pose_deadline - now
            if left > 0:
                cv2.putText(preview, f"CAPTURING: {left:.1f}s", (50, 100), 2, 2, (0, 255, 255), 3)
            else:
                # 倒计时结束，必须抓拍
                print("📸 捕捉照片中...")
                ts = time.strftime("%Y%m%d_%H%M%S")
                target_svg = Path(args.output).with_name(f"pi_draw_{ts}.svg")
                
                # 尝试提取轮廓
                found_shape = False
                if results.masks is not None:
                    masks = results.masks.data.cpu().numpy()
                    raw_mask = (masks[0] > 0.5).astype(np.uint8) * 255
                    m_h, m_w = frame.shape[:2]
                    mask = cv2.resize(raw_mask, (m_w, m_h), interpolation=cv2.INTER_NEAREST)
                    conts, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                    if conts:
                        best_c = max(conts, key=cv2.contourArea).squeeze()
                        save_svg([simplify_contour(best_c, 0.002)], m_w, m_h, target_svg)
                        found_shape = True
                
                if found_shape:
                    cap.release()
                    cv2.destroyAllWindows()
                    
                    print(f"✅ SVG已生成: {target_svg}")
                    target_gcode = target_svg.with_suffix(".ngc")
                    
                    print("🚀 正在自动转换 G-code...")
                    subprocess.run(["python3", "svg-G-code.py", str(target_svg), str(target_gcode)])
                    
                    print("✍️ 启动绘图机...")
                    subprocess.run(["python3", "send.py", str(target_gcode)])
                    
                    print("🎉 绘图已启动，程序退出。")
                    return
                else:
                    print("❌ 未能捕捉到清晰轮廓，重置。")
                    pose_deadline = None
                    person_seen_start = None

        elif has_detection:
            if person_seen_start is None: person_seen_start = now
            if (now - person_seen_start) >= args.auto_trigger_seconds:
                pose_deadline = now + args.pose_seconds
            else:
                cv2.putText(preview, "LOCKED", (50, 100), 2, 2, (0, 255, 0), 3)
        else:
            person_seen_start = None
            cv2.putText(preview, "READY", (50, 100), 2, 2, (255, 255, 255), 2)

        cv2.imshow(win_name, preview)
        if cv2.waitKey(1) & 0xFF == ord('q'): break

    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
