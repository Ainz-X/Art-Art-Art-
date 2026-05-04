#!/usr/bin/env python3
"""
local_display.py – Pi fullscreen display for the AI Art Plotter.

Shows camera image + contour overlay + status text.
YOLO runs in a background thread so the display is always smooth.

Run on the Pi desktop:  python3 ~/src/local_display.py
Or via SSH (Wayland):
    XDG_RUNTIME_DIR=/run/user/1000 WAYLAND_DISPLAY=wayland-0 \
    SDL_VIDEODRIVER=wayland python3 ~/src/local_display.py

Keys: SPACE = manual capture  |  R = reset  |  Q / Esc = quit
"""

from __future__ import annotations

import threading
import time
import subprocess
from pathlib import Path

import cv2
import numpy as np
import pygame

try:
    from ultralytics import YOLO
except ImportError:
    raise SystemExit("ultralytics not installed — run: pip3 install ultralytics")

# ── Config ────────────────────────────────────────────────────────────────────
CAMERA     = 0
CAM_W      = 640           # lower res = faster YOLO + less CPU
CAM_H      = 480
CONF       = 0.25
LOCK_S     = 2.0           # seconds subject must be visible before countdown
POSE_S     = 5.0           # countdown duration (seconds)
EPS        = 0.002         # contour simplification
_SRC       = Path(__file__).parent
MODEL      = str(_SRC / "yolov8n-seg.pt")
OUT_DIR    = _SRC / "output"

# ── Colours (RGB for pygame) ──────────────────────────────────────────────────
BLACK      = (  0,   0,   0)
GREEN      = ( 40, 220, 100)
CYAN       = (  0, 200, 255)
WHITE      = (220, 220, 220)
DIM        = ( 70,  70,  70)
LAVENDER   = (160, 160, 255)
BRIGHT     = ( 80, 255, 120)
RED        = (220,  60,  60)
ORANGE     = (255, 165,   0)

# ── Shared detection state (written by YOLO thread, read by main) ─────────────
_det_lock     = threading.Lock()
_det_frame    = None      # latest BGR frame (np.ndarray)
_det_conts    = []        # list of contour arrays in camera coords
_det_has_det  = False
_det_running  = True

# ── Shared pipeline state (written by plot worker, read by main) ──────────────
_pipe_lock  = threading.Lock()
_pipe_phase = "INIT"      # INIT | READY | DETECTING | COUNTDOWN | CAPTURING | CONVERTING | PLOTTING | DONE | ERROR
_pipe_msg   = "Loading…"


def set_phase(phase: str, msg: str = "") -> None:
    with _pipe_lock:
        global _pipe_phase, _pipe_msg
        _pipe_phase, _pipe_msg = phase, msg


def get_phase() -> tuple[str, str]:
    with _pipe_lock:
        return _pipe_phase, _pipe_msg

# ── Geometry helpers ──────────────────────────────────────────────────────────

def simplify(c: np.ndarray, eps: float) -> np.ndarray:
    c = c.squeeze()
    if c.ndim != 2 or len(c) < 3:
        return c
    peri = cv2.arcLength(c, True)
    return cv2.approxPolyDP(c, peri * eps, True).reshape((-1, 2))


def save_svg(contours: list, w: int, h: int, path: Path) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    lines = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}">']
    for c in contours:
        pts = " ".join(f"{int(x)},{int(y)}" for x, y in c)
        lines.append(f'  <polyline points="{pts}" fill="none" stroke="black" stroke-width="2"/>')
    lines.append("</svg>")
    path.write_text("\n".join(lines))

# ── YOLO detection thread ─────────────────────────────────────────────────────

def _detection_thread(model: YOLO, cap: cv2.VideoCapture) -> None:
    global _det_frame, _det_conts, _det_has_det
    while _det_running:
        ok, frame = cap.read()
        if not ok:
            time.sleep(0.05)
            continue

        try:
            results = model.predict(source=frame, conf=CONF, verbose=False)[0]
        except Exception:
            continue

        has_det = len(results.boxes) > 0
        contours: list = []

        if results.masks is not None:
            fh, fw = frame.shape[:2]
            for m in results.masks.data.cpu().numpy():
                u8 = (m > 0.5).astype(np.uint8) * 255
                u8 = cv2.resize(u8, (fw, fh), interpolation=cv2.INTER_NEAREST)
                cntrs, _ = cv2.findContours(u8, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
                for c in cntrs:
                    s = simplify(c, EPS)
                    if len(s) >= 3:
                        contours.append(s)

        with _det_lock:
            _det_frame   = frame
            _det_conts   = contours
            _det_has_det = has_det

# ── Plot worker (SVG → G-code → plotter) ─────────────────────────────────────

def _plot_worker(contours: list, w: int, h: int) -> None:
    ts  = time.strftime("%Y%m%d_%H%M%S")
    svg = OUT_DIR / f"draw_{ts}.svg"
    ngc = svg.with_suffix(".ngc")

    try:
        set_phase("CONVERTING", "Converting to G-code")
        save_svg(contours, w, h, svg)
        r = subprocess.run(
            ["python3", "svg-G-code.py", str(svg), str(ngc)],
            capture_output=True, text=True, cwd=_SRC,
        )
        if r.returncode != 0:
            set_phase("ERROR", (r.stderr or "Conversion failed").strip()[:70])
            return

        set_phase("PLOTTING", "Sending to plotter")
        r = subprocess.run(
            ["python3", "send.py", str(ngc)],
            capture_output=True, text=True, cwd=_SRC,
            timeout=240,
        )
        if r.returncode != 0:
            details = (r.stdout or "") + "\n" + (r.stderr or "")
            details = " ".join(details.splitlines()).strip()
            set_phase("ERROR", (details or "Plot send failed").strip()[:120])
            return

        set_phase("DONE", "Plotter started")
        time.sleep(5)
        set_phase("READY")

    except subprocess.TimeoutExpired:
        set_phase("ERROR", "Sending to plotter timeout (>240s)")
    except Exception as exc:
        set_phase("ERROR", str(exc)[:70])

# ── pygame drawing helpers ────────────────────────────────────────────────────

def pg_text_center(surface: pygame.Surface, font: pygame.font.Font,
                   text: str, y: int, color: tuple) -> None:
    surf = font.render(text, True, color)
    surface.blit(surf, ((surface.get_width() - surf.get_width()) // 2, y))


def pg_text(surface: pygame.Surface, font: pygame.font.Font,
            text: str, x: int, y: int, color: tuple) -> None:
    surface.blit(font.render(text, True, color), (x, y))


def frame_to_surface(frame: np.ndarray, W: int, H: int) -> pygame.Surface:
    """Convert BGR numpy frame to a full-screen pygame surface."""
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    rgb = cv2.resize(rgb, (W, H), interpolation=cv2.INTER_LINEAR)
    # swapaxes: (H,W,3) → (W,H,3) as pygame surfarray expects
    return pygame.surfarray.make_surface(rgb.swapaxes(0, 1))

# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    global _det_running

    # ── Display init ──────────────────────────────────────────────────────────
    pygame.init()
    pygame.mouse.set_visible(False)
    screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
    pygame.display.set_caption("AI Art Plotter")
    W, H = screen.get_size()

    f_count = pygame.font.SysFont("monospace", min(160, H // 4), bold=True)
    f_large = pygame.font.SysFont("monospace",  48, bold=True)
    f_small = pygame.font.SysFont("monospace",  26)

    # ── Camera + YOLO init (show loading screen) ──────────────────────────────
    screen.fill(BLACK)
    pg_text_center(screen, f_large, "LOADING...", H // 2 - 30, DIM)
    pygame.display.flip()

    model = YOLO(MODEL)
    cap   = cv2.VideoCapture(CAMERA)
    cap.set(cv2.CAP_PROP_FRAME_WIDTH,  CAM_W)
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, CAM_H)

    # Warm-up read
    for _ in range(5):
        cap.read()

    # Scale factors: camera coords → screen coords
    sx, sy = W / CAM_W, H / CAM_H

    # Start detection thread
    det_thread = threading.Thread(
        target=_detection_thread, args=(model, cap), daemon=True
    )
    det_thread.start()

    # Wait for first frame — keep pumping events so Wayland doesn't kill the window
    for _ in range(80):
        pygame.event.pump()          # process OS events without handling them
        screen.fill(BLACK)
        pg_text_center(screen, f_large, "LOADING...", H // 2 - 30, DIM)
        pygame.display.flip()
        with _det_lock:
            if _det_frame is not None:
                break
        time.sleep(0.1)

    set_phase("READY")

    # ── State machine variables ───────────────────────────────────────────────
    person_seen_t: float | None = None
    pose_deadline: float | None = None
    last_contours: list         = []
    manual_trigger: bool        = False

    clock = pygame.time.Clock()
    print(f"Display running at {W}×{H} — SPACE: capture  R: reset  Q/Esc: quit")

    while True:
        # ── Events ────────────────────────────────────────────────────────────
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                _det_running = False; cap.release(); pygame.quit(); return
            elif ev.type == pygame.KEYDOWN:
                if ev.key in (pygame.K_q, pygame.K_ESCAPE):
                    _det_running = False; cap.release(); pygame.quit(); return
                elif ev.key == pygame.K_SPACE:
                    manual_trigger = True
                elif ev.key == pygame.K_r:
                    set_phase("READY")
                    pose_deadline = person_seen_t = None

        phase, msg = get_phase()
        now = time.time()

        # ── Get latest detection ──────────────────────────────────────────────
        with _det_lock:
            frame    = _det_frame
            contours = list(_det_conts)
            has_det  = _det_has_det

        # ── Background: camera frame or black ─────────────────────────────────
        if frame is not None:
            screen.blit(frame_to_surface(frame, W, H), (0, 0))
        else:
            screen.fill(BLACK)

        # ── Contour overlay ───────────────────────────────────────────────────
        draw_conts = contours if contours else last_contours
        for c in draw_conts:
            pts = [(int(x * sx), int(y * sy)) for x, y in c]
            if len(pts) >= 2:
                pygame.draw.lines(screen, GREEN, False, pts, 2)

        if contours:
            last_contours = list(contours)

        # ── Semi-transparent overlay strip for text readability ───────────────
        overlay = pygame.Surface((W, 80), pygame.SRCALPHA)
        overlay.fill((0, 0, 0, 140))
        screen.blit(overlay, (0, 0))

        # ── Work phases (CONVERTING / PLOTTING / DONE / ERROR) ───────────────
        if phase in ("CONVERTING", "PLOTTING", "DONE", "ERROR"):
            if phase == "DONE":
                pg_text_center(screen, f_large, "COMPLETE",       H // 2 - 40, BRIGHT)
                pg_text_center(screen, f_small,  msg,             H // 2 + 20, BRIGHT)
            elif phase == "ERROR":
                pg_text_center(screen, f_large, "ERROR",          H // 2 - 40, RED)
                pg_text_center(screen, f_small,  msg,             H // 2 + 20, RED)
            else:
                dots = "." * (int(now * 1.5) % 4)
                pg_text_center(screen, f_large,  msg + dots,     H // 2 - 30, LAVENDER)

            pg_text(screen, f_small, phase, 20, 30, LAVENDER)
            pygame.display.flip()
            clock.tick(30)
            continue

        # ── Manual trigger ────────────────────────────────────────────────────
        if manual_trigger:
            manual_trigger = False
            pose_deadline  = now
            person_seen_t  = None

        # ── Detection state machine ───────────────────────────────────────────
        if pose_deadline is not None:
            left = pose_deadline - now
            if left > 0:
                pg_text_center(screen, f_count, f"{left:.1f}", H // 2 - 110, CYAN)
                pg_text(screen, f_small, "HOLD POSITION", 20, 30, CYAN)
            else:
                # Flash white
                screen.fill((255, 255, 255))
                pygame.display.flip()
                pygame.time.wait(80)

                pose_deadline = person_seen_t = None
                snap = list(contours or last_contours)
                set_phase("CONVERTING", "Converting to G-code")
                threading.Thread(
                    target=_plot_worker,
                    args=(snap, CAM_W, CAM_H),
                    daemon=True,
                ).start()

        elif has_det:
            if person_seen_t is None:
                person_seen_t = now
            elapsed = now - person_seen_t

            if elapsed >= LOCK_S:
                pose_deadline = now + POSE_S
            else:
                pg_text(screen, f_small, f"DETECTED  {elapsed:.1f}s", 20, 30, GREEN)
                bar_w = int((elapsed / LOCK_S) * (W - 80))
                pygame.draw.rect(screen, GREEN, (40, H - 24, bar_w, 12))

        else:
            person_seen_t = None
            pg_text(screen, f_small, "READY", 20, 30, WHITE)

        pygame.display.flip()
        clock.tick(30)

    _det_running = False
    cap.release()
    pygame.quit()


if __name__ == "__main__":
    main()
