#!/usr/bin/env python3
"""
local_display.py – Lightweight fullscreen display client for the AI Art Plotter.

Does NOT use the camera or YOLO directly.
Reads the live video stream and status from web_app.py (http://localhost:5000).
This eliminates all camera / YOLO conflicts when both processes run simultaneously.

Start web_app.py first, then run this script:
    python3 ~/src/web_app.py &
    XDG_RUNTIME_DIR=/run/user/1000 WAYLAND_DISPLAY=wayland-0 \
        SDL_VIDEODRIVER=wayland python3 ~/src/local_display.py

Keys: SPACE = capture  |  R = reset  |  Q / Esc = quit
"""

from __future__ import annotations

import json
import threading
import time
import urllib.request
from pathlib import Path

import cv2
import numpy as np
import pygame

# ── Config ────────────────────────────────────────────────────────────────────
API_BASE    = "http://127.0.0.1:5000"
VIDEO_URL   = f"{API_BASE}/video_feed"
STATUS_URL  = f"{API_BASE}/api/status"
CAPTURE_URL = f"{API_BASE}/api/capture"
RESET_URL   = f"{API_BASE}/api/reset"

POLL_HZ     = 4          # status polls per second
CONNECT_RETRY = 2.0      # seconds between reconnect attempts

# ── Colours (RGB) ─────────────────────────────────────────────────────────────
BLACK    = (  0,   0,   0)
WHITE    = (220, 220, 220)
DIM      = ( 60,  60,  60)
GREEN    = ( 40, 220, 100)
CYAN     = (  0, 200, 255)
LAVENDER = (160, 160, 255)
BRIGHT   = ( 80, 255, 120)
RED      = (220,  60,  60)
ORANGE   = (255, 165,   0)

# ── Shared buffers ────────────────────────────────────────────────────────────
_running     = True
_frame_lock  = threading.Lock()
_status_lock = threading.Lock()
_frame: np.ndarray | None = None          # latest BGR frame from MJPEG
_status: dict = {
    "status": "CONNECTING",
    "countdown": 0.0,
    "message": "Connecting to web_app…",
    "error": None,
    "last_gcode": None,
}

# ── MJPEG reader thread ───────────────────────────────────────────────────────

def _mjpeg_thread() -> None:
    global _frame
    while _running:
        try:
            stream = urllib.request.urlopen(VIDEO_URL, timeout=5)
            buf = b""
            while _running:
                chunk = stream.read(8192)
                if not chunk:
                    break
                buf += chunk
                # Extract complete JPEG frames from the multipart stream
                while True:
                    start = buf.find(b'\xff\xd8')   # JPEG SOI
                    end   = buf.find(b'\xff\xd9')   # JPEG EOI
                    if start == -1 or end == -1 or end < start:
                        break
                    jpg   = buf[start:end + 2]
                    buf   = buf[end + 2:]
                    arr   = np.frombuffer(jpg, dtype=np.uint8)
                    img   = cv2.imdecode(arr, cv2.IMREAD_COLOR)
                    if img is not None:
                        with _frame_lock:
                            _frame = img
        except Exception:
            with _frame_lock:
                _frame = None
            time.sleep(CONNECT_RETRY)

# ── Status poller thread ──────────────────────────────────────────────────────

def _status_thread() -> None:
    global _status
    while _running:
        try:
            resp = urllib.request.urlopen(STATUS_URL, timeout=2)
            data = json.loads(resp.read().decode())
            with _status_lock:
                _status = data
        except Exception:
            with _status_lock:
                _status["status"] = "CONNECTING"
                _status["message"] = "Waiting for web_app…"
        time.sleep(1.0 / POLL_HZ)

# ── HTTP command helpers ──────────────────────────────────────────────────────

def _post(url: str) -> None:
    try:
        urllib.request.urlopen(
            urllib.request.Request(url, data=b"", method="POST"), timeout=2
        )
    except Exception:
        pass

# ── pygame helpers ────────────────────────────────────────────────────────────

def pg_center(surface: pygame.Surface, font: pygame.font.Font,
              text: str, y: int, color: tuple) -> None:
    surf = font.render(text, True, color)
    surface.blit(surf, ((surface.get_width() - surf.get_width()) // 2, y))


def pg_text(surface: pygame.Surface, font: pygame.font.Font,
            text: str, x: int, y: int, color: tuple) -> None:
    surface.blit(font.render(text, True, color), (x, y))


def frame_to_surface(frame: np.ndarray, W: int, H: int) -> pygame.Surface:
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    rgb = cv2.resize(rgb, (W, H), interpolation=cv2.INTER_LINEAR)
    return pygame.surfarray.make_surface(rgb.swapaxes(0, 1))

# ── Status → display colour ───────────────────────────────────────────────────

STATUS_COLORS = {
    "READY":      WHITE,
    "DETECTING":  GREEN,
    "COUNTDOWN":  CYAN,
    "CAPTURING":  LAVENDER,
    "CONVERTING": LAVENDER,
    "PLOTTING":   LAVENDER,
    "DONE":       BRIGHT,
    "ERROR":      RED,
    "CONNECTING": ORANGE,
}

# ── Main ──────────────────────────────────────────────────────────────────────

def main() -> None:
    global _running

    pygame.init()
    pygame.mouse.set_visible(False)
    screen = pygame.display.set_mode((0, 0), pygame.FULLSCREEN)
    pygame.display.set_caption("AI Art Plotter")
    W, H = screen.get_size()

    f_count = pygame.font.SysFont("monospace", min(160, H // 4), bold=True)
    f_large = pygame.font.SysFont("monospace",  48, bold=True)
    f_small = pygame.font.SysFont("monospace",  26)

    # Start background threads
    threading.Thread(target=_mjpeg_thread,  daemon=True).start()
    threading.Thread(target=_status_thread, daemon=True).start()

    clock = pygame.time.Clock()
    print(f"Display client running at {W}×{H} — SPACE: capture  R: reset  Q/Esc: quit")

    while True:
        # ── Events ────────────────────────────────────────────────────────────
        for ev in pygame.event.get():
            if ev.type == pygame.QUIT:
                _running = False; pygame.quit(); return
            elif ev.type == pygame.KEYDOWN:
                if ev.key in (pygame.K_q, pygame.K_ESCAPE):
                    _running = False; pygame.quit(); return
                elif ev.key == pygame.K_SPACE:
                    threading.Thread(target=_post, args=(CAPTURE_URL,), daemon=True).start()
                elif ev.key == pygame.K_r:
                    threading.Thread(target=_post, args=(RESET_URL,),   daemon=True).start()

        # ── Fetch latest frame + status ───────────────────────────────────────
        with _frame_lock:
            frame = _frame

        with _status_lock:
            st = dict(_status)

        phase    = st.get("status",    "CONNECTING")
        msg      = st.get("message",   "")
        error    = st.get("error",     None)
        countdown = float(st.get("countdown", 0.0))
        color    = STATUS_COLORS.get(phase, WHITE)
        now      = time.time()

        # ── Draw background ───────────────────────────────────────────────────
        if frame is not None:
            screen.blit(frame_to_surface(frame, W, H), (0, 0))
        else:
            screen.fill(BLACK)
            pg_center(screen, f_large, "Connecting to web_app…", H // 2, ORANGE)
            pygame.display.flip()
            clock.tick(10)
            continue

        # ── Semi-transparent top bar ──────────────────────────────────────────
        bar = pygame.Surface((W, 70), pygame.SRCALPHA)
        bar.fill((0, 0, 0, 160))
        screen.blit(bar, (0, 0))

        # ── Status text ───────────────────────────────────────────────────────
        label = error if (phase == "ERROR" and error) else msg
        pg_text(screen,  f_small, phase,  20, 16, color)
        pg_text(screen,  f_small, label,  20, 44, color)

        # ── Big countdown number ──────────────────────────────────────────────
        if phase == "COUNTDOWN" and countdown > 0:
            pg_center(screen, f_count, f"{countdown:.1f}", H // 2 - 110, CYAN)

        # ── Work phase centre message ─────────────────────────────────────────
        elif phase in ("CONVERTING", "PLOTTING"):
            dots = "." * (int(now * 1.5) % 4)
            pg_center(screen, f_large, label + dots, H // 2 - 30, LAVENDER)

        elif phase == "DONE":
            pg_center(screen, f_large, "COMPLETE",  H // 2 - 50, BRIGHT)

        elif phase == "ERROR":
            pg_center(screen, f_large, "ERROR",     H // 2 - 50, RED)
            if error:
                pg_center(screen, f_small, error[:70], H // 2 + 20, RED)

        # ── Detection progress bar ────────────────────────────────────────────
        if phase == "DETECTING":
            frac = min(1.0, countdown / 2.0) if countdown else 0  # countdown reused as elapsed here
            pygame.draw.rect(screen, GREEN, (40, H - 24, int((W - 80) * frac), 12))

        pygame.display.flip()
        clock.tick(30)

    _running = False
    pygame.quit()


if __name__ == "__main__":
    main()
