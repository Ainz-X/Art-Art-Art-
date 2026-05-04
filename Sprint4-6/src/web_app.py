#!/usr/bin/env python3
"""
web_app.py – Browser-based UI for the AI Art Plotter.

Run on the Raspberry Pi:
    python3 web_app.py

Then open from any device on the same WiFi network:
    http://<pi-ip-address>:5000
    (find the Pi's IP with: hostname -I)
"""

from __future__ import annotations

import importlib.util
import os
import threading
import time
from pathlib import Path

import cv2
import numpy as np
from flask import Flask, Response, jsonify, render_template, request

# ── Load existing modules without modifying them ──────────────────────────────

_BASE = Path(__file__).parent


def _load_module(name: str, filename: str):
    import sys
    spec = importlib.util.spec_from_file_location(name, _BASE / filename)
    mod = importlib.util.module_from_spec(spec)
    sys.modules[name] = mod          # must register before exec so dataclasses can resolve __module__
    spec.loader.exec_module(mod)
    return mod


_svg_gcode = _load_module("svg_gcode", "svg-G-code.py")
_send_mod = _load_module("send_mod", "send.py")

try:
    from ultralytics import YOLO
    _YOLO_OK = True
except ImportError:
    _YOLO_OK = False

# ── Flask app ─────────────────────────────────────────────────────────────────

app = Flask(__name__)

# ── Shared state (always access inside _lock) ─────────────────────────────────

_lock = threading.Lock()

_state: dict = {
    "status": "READY",
    # READY | DETECTING | COUNTDOWN | CAPTURING | CONVERTING | PLOTTING | HOMING | DONE | ERROR
    "countdown": 0.0,
    "message": "Waiting for subject…",
    "last_svg": None,
    "last_gcode": None,
    "error": None,
}

_settings: dict = {
    # Camera / YOLO
    "camera_index": 0,
    "cam_width": 1280,
    "cam_height": 720,
    "model": "yolov8n-seg.pt",
    "conf": 0.25,
    "auto_trigger_seconds": 2.0,
    "pose_seconds": 5.0,
    # Plot dimensions (mm)  — default A4 portrait (desired drawing box)
    "target_width_mm":  210.0,
    "target_height_mm": 297.0,
    # Physical travel limits — must match your plotter; svg-G-code clamps targets to these
    "machine_max_width_mm":  280.0,
    "machine_max_height_mm": 200.0,
    # Inset from each edge when scaling (mm), keeps pen off hard stops
    "machine_margin_mm": 3.0,
    # Extra shrink of the *target* box (0.05–1). 0.5 = half size on bed before machine clamp.
    "plot_content_scale": 0.5,
    # Whole drawing shifted right in mm (e.g. 20 = 2 cm away from left limit)
    "plot_offset_x_mm": 10.0,
    # Display
    "mirror": False,
    "edge_only": True,
    # Feed rates (mm/min)
    "travel_feed": 3000,
    "draw_feed": 1500,
    "pen_feed": 800,
    # Pen Z positions (mm) — for Z-axis plotters
    "pen_up_z": 5.0,
    "pen_down_z": 0.0,
    # Servo pen control — for servo-based plotters (Pi's current svg-G-code.py)
    "pen_up_servo": 100,
    "pen_down_servo": 500,
    # Plotter kinematics
    "plotter_type": "cartesian",    # cartesian | corexy | polar
    # Serial
    "usb_port": "/dev/ttyUSB0",
    "baud_rate": 115200,
    # Workflow
    "auto_plot": True,
    # Before $H homing: try M3 pen-up (servo plotters). Many plain GRBL builds reject M3 — ignored on failure.
    "homing_try_pen_up": True,
}

# Latest JPEG bytes for each live stream
_frames: dict[str, bytes | None] = {"camera": None, "edge": None}

_manual_capture   = threading.Event()
_stop_camera      = threading.Event()
_stop_plot        = threading.Event()      # set to abort a running plot
_last_edge_frame: np.ndarray | None = None # last processed edge frame (black+contours)
_cooldown_until:  float = 0.0              # no auto-detection before this timestamp
_reset_home_lock  = threading.Lock()       # only one reset/homing at a time

# ── Geometry helpers ──────────────────────────────────────────────────────────

def _simplify(contour: np.ndarray, eps: float) -> np.ndarray:
    if contour.shape[0] < 3 or eps <= 0:
        return contour
    peri = cv2.arcLength(contour, True)
    return cv2.approxPolyDP(contour, peri * eps, True).reshape((-1, 2))


def _write_svg(contours: list, w: int, h: int, path: Path) -> None:
    lines = [f'<svg xmlns="http://www.w3.org/2000/svg" width="{w}" height="{h}">']
    for c in contours:
        pts = " ".join(f"{int(x)},{int(y)}" for x, y in c)
        lines.append(f'  <polyline points="{pts}" fill="none" stroke="black" stroke-width="2"/>')
    lines.append("</svg>")
    path.write_text("\n".join(lines), encoding="utf-8")


def _polar_postprocess(gcode_path: Path) -> None:
    """Rewrite X/Y Cartesian coordinates as radius/angle for polar plotters."""
    import math
    out: list[str] = []
    for line in gcode_path.read_text().splitlines():
        code = line.split(";")[0].strip()
        if code.startswith(("G0 ", "G1 ")) and "X" in code and "Y" in code:
            params: dict[str, float] = {}
            for token in code.split()[1:]:
                if len(token) > 1 and token[0].isalpha():
                    try:
                        params[token[0]] = float(token[1:])
                    except ValueError:
                        pass
            if "X" in params and "Y" in params:
                r = math.hypot(params["X"], params["Y"])
                theta = math.degrees(math.atan2(params["Y"], params["X"]))
                rebuilt = [code.split()[0], f"X{r:.3f}", f"Y{theta:.3f}"]
                if "Z" in params:
                    rebuilt.append(f"Z{params['Z']:.3f}")
                if "F" in params:
                    rebuilt.append(f"F{int(params['F'])}")
                out.append(" ".join(rebuilt))
                continue
        out.append(line)
    gcode_path.write_text("\n".join(out), encoding="utf-8")

# ── Camera / detection background thread ─────────────────────────────────────

def _camera_thread() -> None:
    if not _YOLO_OK:
        with _lock:
            _state["status"] = "ERROR"
            _state["error"] = "ultralytics not installed — run: pip install ultralytics"
        return

    with _lock:
        s0 = dict(_settings)

    model = YOLO(s0["model"])
    cap = cv2.VideoCapture(s0["camera_index"])
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, s0["cam_width"])
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, s0["cam_height"])

    global _last_edge_frame, _cooldown_until
    person_seen_t: float | None = None
    pose_deadline: float | None = None
    font = cv2.FONT_HERSHEY_SIMPLEX

    while not _stop_camera.is_set():
        ok, raw = cap.read()
        if not ok:
            time.sleep(0.05)
            continue

        with _lock:
            s = dict(_settings)
            cur_status = _state["status"]

        # ── YOLO: only detect people (class 0), always runs ───────────────
        results = model.predict(source=raw, conf=s["conf"], classes=[0], verbose=False)[0]
        now = time.time()
        has_det = len(results.boxes) > 0

        display = cv2.flip(raw, 1) if s["mirror"] else raw.copy()
        edge = np.ones_like(display) * 255   # white background

        # Draw only the largest person contour (matches original main.py logic)
        if results.masks is not None:
            h, w = raw.shape[:2]
            raw_mask = (results.masks.data.cpu().numpy()[0] > 0.5).astype(np.uint8) * 255
            mask = cv2.resize(raw_mask, (w, h), interpolation=cv2.INTER_NEAREST)
            cntrs, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            if cntrs:
                best = max(cntrs, key=cv2.contourArea)
                cv2.drawContours(edge, [best], -1, (0, 0, 0), 3)

        # ── Capture state machine — only when idle ─────────────────────────
        busy = cur_status in ("CAPTURING", "CONVERTING", "PLOTTING", "HOMING")

        if not busy:
            if _manual_capture.is_set():
                _manual_capture.clear()
                pose_deadline = now
                person_seen_t = None

            if pose_deadline is not None:
                left = pose_deadline - now
                if left > 0:
                    with _lock:
                        _state.update({"status": "COUNTDOWN", "countdown": left,
                                       "message": f"Capturing in {left:.1f}s…"})
                    label = f"{left:.1f}s"
                    cv2.putText(edge, label, (40, 90), font, 2.0, (0, 0, 0), 3)
                else:
                    pose_deadline = None
                    person_seen_t = None
                    threading.Thread(
                        target=_capture_worker,
                        args=(raw.copy(), results, dict(s)),
                        daemon=True,
                    ).start()

            elif has_det:
                if now < _cooldown_until:
                    person_seen_t = None   # cooldown after a job
                else:
                    if person_seen_t is None:
                        person_seen_t = now
                    elapsed = now - person_seen_t
                    if elapsed >= s["auto_trigger_seconds"]:
                        pose_deadline = now + s["pose_seconds"]
                    else:
                        with _lock:
                            _state.update({
                                "status": "DETECTING",
                                "message": f"Hold still… {elapsed:.1f} / {s['auto_trigger_seconds']}s",
                            })

            else:
                person_seen_t = None
                with _lock:
                    if _state["status"] in ("READY", "DETECTING", "COUNTDOWN"):
                        _state.update({"status": "READY", "countdown": 0.0,
                                       "message": "Waiting for subject…"})

        _last_edge_frame = edge.copy()
        _encode_and_store(display, edge, s)

    cap.release()


def _encode_and_store(camera: np.ndarray, edge: np.ndarray, _s: dict) -> None:
    enc = [cv2.IMWRITE_JPEG_QUALITY, 72]
    _, cj = cv2.imencode(".jpg", camera, enc)
    _, ej = cv2.imencode(".jpg", edge,   enc)
    with _lock:
        _frames["camera"] = cj.tobytes()
        _frames["edge"]   = ej.tobytes()

# ── Capture → convert → plot pipeline (runs in daemon threads) ────────────────

def _capture_worker(frame: np.ndarray, results, s: dict) -> None:
    with _lock:
        _state.update({"status": "CAPTURING", "message": "Extracting outline…"})

    h, w = frame.shape[:2]
    out_dir = _BASE / "output"
    out_dir.mkdir(exist_ok=True)
    ts = time.strftime("%Y%m%d_%H%M%S")
    svg_path = out_dir / f"web_{ts}.svg"

    found = False
    if results.masks is not None:
        raw_mask = (results.masks.data.cpu().numpy()[0] > 0.5).astype(np.uint8) * 255
        mask = cv2.resize(raw_mask, (w, h), interpolation=cv2.INTER_NEAREST)
        cntrs, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if cntrs:
            best = max(cntrs, key=cv2.contourArea).squeeze()
            pts = _simplify(best, 0.002)
            if s["mirror"]:
                pts = np.column_stack([w - pts[:, 0], pts[:, 1]])
            # Rotate contour so it starts from the point closest to G-code (0,0).
            # G-code origin = bottom-left of drawing = SVG pixel (min_x, max_y).
            ox = float(pts[:, 0].min())
            oy = float(pts[:, 1].max())
            start = int(np.hypot(pts[:, 0] - ox, pts[:, 1] - oy).argmin())
            pts = np.roll(pts, -start, axis=0)
            _write_svg([pts], w, h, svg_path)
            found = True

    if not found:
        with _lock:
            _state.update({"status": "ERROR", "error": "No clear outline — try again."})
        return

    with _lock:
        _state.update({"last_svg": str(svg_path),
                        "status": "CONVERTING",
                        "message": "Converting SVG → G-code…"})

    gcode_path = svg_path.with_suffix(".ngc")
    try:
        # Build Config with only the fields the Pi's svg-G-code.py actually has.
        # That version uses servo pen control (pen_up_servo / pen_down_servo)
        # instead of Z-axis (pen_up_z / pen_down_z).
        import inspect as _inspect
        _cfg_params = set(_inspect.signature(_svg_gcode.Config).parameters)

        w_mm = float(s.get("target_width_mm") or 210)
        h_mm = float(s.get("target_height_mm") or 297)
        cs = float(s.get("plot_content_scale", 0.5))
        cs = max(0.05, min(1.0, cs))
        w_mm *= cs
        h_mm *= cs

        cfg_kwargs: dict = {
            "target_width_mm":  w_mm,
            "target_height_mm": h_mm,
            "travel_feed":      int(s["travel_feed"]),
            "draw_feed":        int(s["draw_feed"]),
        }
        max_w = float(s.get("machine_max_width_mm", 280))
        max_h = float(s.get("machine_max_height_mm", 200))
        margin = max(0.0, float(s.get("machine_margin_mm", 3.0)))
        if "margin_mm" in _cfg_params:
            cfg_kwargs["margin_mm"] = margin
        if "max_width_mm" in _cfg_params:
            cfg_kwargs["max_width_mm"] = max_w
        if "max_height_mm" in _cfg_params:
            cfg_kwargs["max_height_mm"] = max_h
        # Servo or Z pen control
        if "pen_up_servo"   in _cfg_params: cfg_kwargs["pen_up_servo"]   = int(s.get("pen_up_servo",   100))
        if "pen_down_servo" in _cfg_params: cfg_kwargs["pen_down_servo"] = int(s.get("pen_down_servo", 500))
        if "pen_up_z"       in _cfg_params: cfg_kwargs["pen_up_z"]       = float(s.get("pen_up_z",   5.0))
        if "pen_down_z"     in _cfg_params: cfg_kwargs["pen_down_z"]     = float(s.get("pen_down_z", 0.0))
        if "pen_feed"       in _cfg_params: cfg_kwargs["pen_feed"]       = int(s.get("pen_feed",    800))
        if "plot_offset_x_mm" in _cfg_params:
            cfg_kwargs["plot_offset_x_mm"] = float(s.get("plot_offset_x_mm", 10.0))
        cfg = _svg_gcode.Config(**cfg_kwargs)
        _svg_gcode.convert_svg_to_gcode(str(svg_path), str(gcode_path), cfg)
        if s["plotter_type"] == "polar":
            _polar_postprocess(gcode_path)
    except Exception as exc:
        with _lock:
            _state.update({"status": "ERROR", "error": str(exc)})
        return

    with _lock:
        _state["last_gcode"] = str(gcode_path)

    if s["auto_plot"]:
        _run_plot(str(gcode_path), s)
    else:
        with _lock:
            _state.update({"status": "DONE",
                            "message": "G-code ready — click Plot Last to send."})


def _grbl_send(ser, line: str) -> bool:
    """Send one G-code line, wait for GRBL 'ok'. Returns False if _stop_plot set."""
    ser.write((line + "\n").encode("utf-8"))
    deadline = time.time() + 30
    while time.time() < deadline:
        if _stop_plot.is_set():
            return False
        try:
            resp = ser.readline().decode("utf-8", errors="ignore").strip()
        except Exception:
            time.sleep(0.05)
            continue
        if not resp:
            continue
        if resp.lower() == "ok":
            return True
        if resp.lower().startswith("error"):
            print(f"[plotter] {line!r} → {resp}")
            return True   # log but continue
    return True   # timeout — continue anyway


def _grbl_init(ser) -> None:
    """Wake GRBL, clear buffer, unlock alarm."""
    ser.write(b"\r\n\r\n")
    time.sleep(2)
    ser.reset_input_buffer()
    ser.write(b"$X\n")
    time.sleep(0.3)
    ser.reset_input_buffer()


def _border_test(s: dict) -> None:
    """Draw a rectangle at the configured plot size so the user can verify paper placement."""
    import serial as _serial

    port     = s.get("usb_port",      "/dev/ttyUSB0")
    baud     = int(s.get("baud_rate",  115200))
    max_w = float(s.get("machine_max_width_mm", 280))
    max_h = float(s.get("machine_max_height_mm", 200))
    w = min(float(s.get("target_width_mm") or 210), max_w)
    h = min(float(s.get("target_height_mm") or s.get("target_width_mm") or 210), max_h)
    tf       = int(s.get("travel_feed",  3000))
    df       = int(s.get("draw_feed",    1500))
    pen_up   = int(s.get("pen_up_servo",  100))
    pen_down = int(s.get("pen_down_servo", 500))

    if not os.path.exists(port):
        return

    _stop_plot.clear()
    with _lock:
        _state.update({"status": "PLOTTING",
                        "message": f"Border test {w:.0f}×{h:.0f} mm…"})
    try:
        ser = _serial.Serial(port, baud, timeout=5)
        _grbl_init(ser)

        cmds = [
            "G21", "G90",
            f"M3 S{pen_up}", "G4 P0.500",
            f"G0 X0.000 Y0.000 F{tf}",
            f"M3 S{pen_down}", "G4 P0.500",
            f"G1 X{w:.3f} Y0.000 F{df}",
            f"G1 X{w:.3f} Y{h:.3f} F{df}",
            f"G1 X0.000 Y{h:.3f} F{df}",
            f"G1 X0.000 Y0.000 F{df}",
            f"M3 S{pen_up}", "G4 P0.500",
            f"G0 X0.000 Y0.000 F{tf}",
            "M2",
        ]
        for cmd in cmds:
            if not _grbl_send(ser, cmd):
                ser.write(b"\x18")
                break
        ser.close()
        with _lock:
            _state.update({"status": "DONE", "message": "Border test complete!"})
    except Exception as exc:
        with _lock:
            _state.update({"status": "ERROR", "error": f"Border test: {exc}"})


def _auto_reset(delay: float = 5.0) -> None:
    """Wait delay seconds, return to READY with a 15-second cooldown before next detection."""
    global _cooldown_until  # noqa: PLW0603
    time.sleep(delay)
    _cooldown_until = time.time() + 15.0   # 15s pause before next capture
    with _lock:
        if _state["status"] in ("DONE", "ERROR"):
            _state.update({"status": "READY", "countdown": 0.0,
                            "message": "Waiting for subject…", "error": None})


def _run_plot(gcode_file: str, s: dict) -> None:
    """Send G-code to plotter. Pen up → home (0,0) → run file → home."""
    import serial as _serial

    port     = s.get("usb_port",       "/dev/ttyUSB0")
    baud     = int(s.get("baud_rate",   115200))
    pen_up   = int(s.get("pen_up_servo", 100))
    tf       = int(s.get("travel_feed",  3000))

    _stop_plot.clear()

    with _lock:
        _state.update({"status": "PLOTTING", "message": "Connecting to plotter…"})

    if not os.path.exists(port):
        with _lock:
            _state.update({"status": "ERROR",
                            "error": f"{port} not found — is the plotter connected?"})
        return

    ser = None
    try:
        ser = _serial.Serial(port, baud, timeout=5)
        _grbl_init(ser)

        # ── Pen up → go to origin before starting ────────────────────────────
        for cmd in [f"M3 S{pen_up}", "G4 P0.500", f"G0 X0.000 Y0.000 F{tf}"]:
            if not _grbl_send(ser, cmd):
                ser.write(b"\x18"); ser.close()
                with _lock:
                    _state.update({"status": "READY", "countdown": 0.0,
                                    "message": "Stopped.", "error": None})
                return

        # ── Send G-code file ──────────────────────────────────────────────────
        with open(gcode_file, "r") as f:
            lines = [l.strip() for l in f
                     if l.strip() and not l.strip().startswith(";")]
        total = len(lines)

        stopped = False
        for i, line in enumerate(lines):
            with _lock:
                _state.update({"status": "PLOTTING",
                                "message": f"Line {i + 1} / {total}"})
            if not _grbl_send(ser, line):
                stopped = True
                break

        if stopped:
            ser.write(b"\x18")
            time.sleep(0.5)
            ser.close()
            with _lock:
                _state.update({"status": "READY", "countdown": 0.0,
                                "message": "Stopped — ready for next capture.",
                                "error": None})
        else:
            ser.close()
            with _lock:
                _state.update({"status": "DONE", "message": "Plot complete!"})
            threading.Thread(target=_auto_reset, daemon=True).start()

    except _serial.SerialException as exc:
        if ser:
            try: ser.close()
            except Exception: pass
        with _lock:
            _state.update({"status": "ERROR", "error": str(exc)})
        threading.Thread(target=_auto_reset, daemon=True).start()
    except Exception as exc:
        if ser:
            try: ser.close()
            except Exception: pass
        with _lock:
            _state.update({"status": "ERROR", "error": str(exc)})
        threading.Thread(target=_auto_reset, daemon=True).start()


def _grbl_wait_ok_long(ser, timeout_s: float) -> bool:
    """Like send.py wait_for_ok — tolerates Grbl status lines; stoppable via _stop_plot."""
    start = time.time()
    while (time.time() - start) < timeout_s:
        if _stop_plot.is_set():
            return False
        try:
            line = ser.readline().decode("utf-8", errors="ignore").strip()
        except Exception:
            time.sleep(0.05)
            continue
        if not line:
            continue
        lower = line.lower()
        if lower == "ok" or lower.startswith("ok ") or "ok" in lower:
            return True
        if "error" in lower or "alarm" in lower:
            return False
        if lower.startswith("<") or lower.startswith("[msg:"):
            continue
    return False


def _grbl_homing_worker(s: dict) -> None:
    """$H homing after optional pen-up; releases _stop_plot when done."""
    if not _reset_home_lock.acquire(blocking=False):
        return
    import serial as _serial

    port = s.get("usb_port", "/dev/ttyUSB0")
    baud = int(s.get("baud_rate", 115200))
    pen_up = int(s.get("pen_up_servo", 100))
    home_timeout = float(getattr(_send_mod, "HOME_TIMEOUT_S", 90))

    time.sleep(1.2)  # let an aborted plot close the serial port first

    try:
        with _lock:
            _state.update({"status": "HOMING", "message": "Homing plotter ($H)…", "error": None})

        if not os.path.exists(port):
            with _lock:
                _state.update({"status": "READY", "countdown": 0.0,
                                "message": "Waiting for subject…",
                                "error": f"{port} not found — homing skipped."})
            return

        ser = None
        try:
            ser = _serial.Serial(port, baud, timeout=1, write_timeout=2)
            _grbl_init(ser)
            # Optional pen-up before $H — servo plotters; stock GRBL often errors on M3 (ignored).
            if s.get("homing_try_pen_up", True):
                for cmd in (f"M3 S{pen_up}", "G4 P0.500"):
                    ser.write((cmd + "\n").encode("utf-8"))
                    if not _grbl_wait_ok_long(ser, 15.0):
                        try:
                            ser.reset_input_buffer()
                        except Exception:
                            pass
                        break
            ser.write(b"$H\n")
            if not _grbl_wait_ok_long(ser, home_timeout):
                raise RuntimeError("Homing ($H) failed, timed out, or stopped")
        finally:
            if ser is not None:
                try:
                    ser.close()
                except Exception:
                    pass

        with _lock:
            _state.update({"status": "READY", "countdown": 0.0,
                            "message": "Homing complete — waiting for subject…",
                            "error": None})
    except Exception as exc:
        with _lock:
            _state.update({"status": "ERROR", "error": str(exc)})
        threading.Thread(target=_auto_reset, daemon=True).start()
    finally:
        _stop_plot.clear()
        _reset_home_lock.release()


# ── Flask routes ──────────────────────────────────────────────────────────────

def _mjpeg_stream(key: str):
    while True:
        with _lock:
            data = _frames.get(key)
        if data:
            yield b"--frame\r\nContent-Type: image/jpeg\r\n\r\n" + data + b"\r\n"
        time.sleep(0.033)   # ~30 fps cap


@app.route("/")
def index():
    return render_template("index.html")


@app.route("/video_feed")
def video_feed():
    return Response(_mjpeg_stream("camera"),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/edge_feed")
def edge_feed():
    return Response(_mjpeg_stream("edge"),
                    mimetype="multipart/x-mixed-replace; boundary=frame")


@app.route("/api/status")
def api_status():
    with _lock:
        return jsonify(dict(_state))


_BOOL_KEYS  = {"mirror", "edge_only", "auto_plot", "homing_try_pen_up"}
_INT_KEYS   = {"camera_index", "cam_width", "cam_height",
               "travel_feed", "draw_feed", "pen_feed", "baud_rate"}
_FLOAT_KEYS = {"conf", "auto_trigger_seconds", "pose_seconds",
               "target_width_mm", "target_height_mm", "pen_up_z", "pen_down_z",
               "machine_max_width_mm", "machine_max_height_mm", "machine_margin_mm",
               "plot_content_scale", "plot_offset_x_mm"}


@app.route("/api/settings", methods=["GET", "POST"])
def api_settings():
    if request.method == "GET":
        with _lock:
            return jsonify(dict(_settings))

    body: dict = request.get_json(force=True)
    with _lock:
        for k, v in body.items():
            if k not in _settings:
                continue
            if v is None:
                _settings[k] = None
            elif k in _BOOL_KEYS:
                _settings[k] = bool(v)
            elif k in _INT_KEYS:
                _settings[k] = int(float(str(v)))
            elif k in _FLOAT_KEYS:
                _settings[k] = float(str(v))
            else:
                _settings[k] = str(v)
    return jsonify({"ok": True})


@app.route("/api/capture", methods=["POST"])
def api_capture():
    _manual_capture.set()
    return jsonify({"ok": True})


@app.route("/api/plot", methods=["POST"])
def api_plot():
    with _lock:
        gcode = _state.get("last_gcode")
        s = dict(_settings)
    if not gcode or not os.path.exists(gcode):
        return jsonify({"error": "No G-code file available"}), 400
    threading.Thread(target=_run_plot, args=(gcode, s), daemon=True).start()
    return jsonify({"ok": True})


@app.route("/api/setzero", methods=["POST"])
def api_setzero():
    """Set current plotter position as (0, 0) using G92."""
    import serial as _serial
    with _lock:
        s = dict(_settings)
    port = s.get("usb_port", "/dev/ttyUSB0")
    baud = int(s.get("baud_rate", 115200))
    if not os.path.exists(port):
        return jsonify({"error": f"{port} not found"}), 400
    try:
        ser = _serial.Serial(port, baud, timeout=3)
        ser.write(b"G92 X0 Y0\n")
        time.sleep(0.3)
        ser.close()
        return jsonify({"ok": True})
    except Exception as exc:
        return jsonify({"error": str(exc)}), 500


@app.route("/api/border_test", methods=["POST"])
def api_border_test():
    with _lock:
        s = dict(_settings)
    threading.Thread(target=_border_test, args=(s,), daemon=True).start()
    return jsonify({"ok": True})


@app.route("/api/stop", methods=["POST"])
def api_stop():
    """Abort the current plot immediately."""
    _stop_plot.set()
    return jsonify({"ok": True})


@app.route("/api/reset", methods=["POST"])
def api_reset():
    """Abort any active plot, then run Grbl homing ($H) in the background."""
    _stop_plot.set()
    with _lock:
        s = dict(_settings)
        _state.update({"status": "HOMING", "countdown": 0.0,
                        "message": "Stopping plot / homing ($H)…", "error": None})
    threading.Thread(target=_grbl_homing_worker, args=(s,), daemon=True).start()
    return jsonify({"ok": True})

# ── Entry point ───────────────────────────────────────────────────────────────

if __name__ == "__main__":
    threading.Thread(target=_camera_thread, daemon=True).start()

    def _startup_border():
        time.sleep(6)          # let Flask and camera init first
        with _lock:
            s = dict(_settings)
        if os.path.exists(s.get("usb_port", "/dev/ttyUSB0")):
            _border_test(s)

    threading.Thread(target=_startup_border, daemon=True).start()

    print()
    print("  ┌─────────────────────────────────────────┐")
    print("  │   AI Art Plotter — Web Interface        │")
    print("  ├─────────────────────────────────────────┤")
    print("  │   Local:   http://localhost:5000        │")
    print("  │   Network: http://<this-pi-ip>:5000     │")
    print("  │            (find IP with: hostname -I)  │")
    print("  └─────────────────────────────────────────┘")
    print()

    app.run(host="0.0.0.0", port=5000, debug=False, threaded=True)
