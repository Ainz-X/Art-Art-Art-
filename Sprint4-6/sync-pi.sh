#!/usr/bin/env bash
# Sync this repo's src/ to the Pi's ~/src/ (same layout as start.sh / local_display.py).
# Prerequisites: Pi 上已开 SSH；本机与 Pi 同一局域网（Wi‑Fi）。
#
# 默认同步到 artartart@artartart.local（mDNS；IP 会变时仍可用）。
# 首次 rsync/ssh 会提示输入登录密码；配置免密后可省略：
#   ssh-copy-id artartart@artartart.local
#
# 覆盖目标主机：
#   PI=artartart@192.168.x.x bash sync-pi.sh

set -euo pipefail
ROOT="$(cd "$(dirname "$0")" && pwd)"
PI="${PI:-artartart@artartart.local}"

rsync -avz \
  --exclude '__pycache__/' \
  --exclude '*.pyc' \
  -e ssh \
  "$ROOT/src/" "$PI:~/src/"

echo "Synced $ROOT/src/ -> $PI:~/src/"
