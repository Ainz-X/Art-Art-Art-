#!/bin/bash
# start.sh – Launch the AI Art Plotter local display
# Run this on the Pi:  bash ~/src/start.sh
# Or make executable:  chmod +x ~/src/start.sh && ~/src/start.sh

cd "$(dirname "$0")"

# Auto-detect display environment
if [ -n "$WAYLAND_DISPLAY" ]; then
    # Already in a Wayland session — use as-is
    echo "Using existing Wayland display: $WAYLAND_DISPLAY"

elif [ -S "/run/user/1000/wayland-0" ]; then
    # Wayland socket exists — set env vars for SSH sessions
    export XDG_RUNTIME_DIR=/run/user/1000
    export WAYLAND_DISPLAY=wayland-0
    export SDL_VIDEODRIVER=wayland
    echo "Detected Wayland — using wayland-0"

elif [ -n "$DISPLAY" ]; then
    # X11 session already active
    echo "Using existing X11 display: $DISPLAY"

elif [ -S "/tmp/.X11-unix/X0" ]; then
    # X11 socket exists (XWayland or plain X)
    export DISPLAY=:0
    export XAUTHORITY=/home/artartart/.Xauthority
    echo "Detected X11 — using :0"

else
    echo "ERROR: No display found. Run this script directly on the Pi desktop."
    exit 1
fi

echo "Starting AI Art Plotter..."
python3 local_display.py
