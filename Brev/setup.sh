#!/usr/bin/env bash
# Brev box setup: serves the vision LLM that Raspberry/vision.py (ask_llm) calls.
# Idempotent: re-run after a fresh instance or a crash, it only does what is missing.
#
#   bash Brev/setup.sh         install (first run only), start server, wait until ready
#   bash Brev/setup.sh stop    stop server
#
# MODEL must equal "brev_model" and PORT the port in "brev_url" in Raspberry/config.json.
set -euo pipefail

MODEL=${MODEL:-Qwen/Qwen2.5-VL-7B-Instruct}
PORT=${PORT:-8000}
VENV=$HOME/vllm-venv
LOG=$HOME/vllm.log
KEYFILE=$HOME/.brev_key

if [ "${1:-}" = stop ]; then
    pkill -f "vllm serve" && echo "stopped" || echo "was not running"
    exit 0
fi

command -v nvidia-smi >/dev/null || { echo "no GPU driver (nvidia-smi missing): wrong instance type?"; exit 1; }
nvidia-smi --query-gpu=name,memory.total --format=csv,noheader   # 7B in bf16 needs ~24 GB

# Python: the box may ship without it or without venv/pip. python-is-python3 makes plain `python` work everywhere.
PKGS="python3 python3-venv python3-pip python-is-python3"
dpkg -s $PKGS >/dev/null 2>&1 || { sudo apt-get update -qq; sudo apt-get install -y $PKGS; }

# Permanent shell aliases (new shells; run `source ~/.bashrc` for the current one)
grep -q "# brev-aliases" "$HOME/.bashrc" 2>/dev/null || cat >> "$HOME/.bashrc" <<'EOF'
# brev-aliases
alias python=python3
alias cls=clear
alias clc=clear
EOF

# vllm brings its own torch+CUDA
[ -d "$VENV" ] || python3 -m venv "$VENV"
[ -x "$VENV/bin/vllm" ] || "$VENV/bin/pip" install -q -U pip vllm

# The port is reachable from the internet, so the server always has a token (vision.py sends $BREV_KEY).
[ -z "${BREV_KEY:-}" ] || echo "$BREV_KEY" > "$KEYFILE"
[ -f "$KEYFILE" ] || (umask 077; openssl rand -hex 16 > "$KEYFILE")
KEY=$(cat "$KEYFILE")

up() { curl -sf -H "Authorization: Bearer $KEY" "localhost:$PORT/v1/models" >/dev/null; }

if up; then
    echo "server already running"
else
    # 8192 tokens is plenty: one 896x672 photo is ~770 image tokens + the prompt + a JSON answer
    nohup setsid "$VENV/bin/vllm" serve "$MODEL" --host 0.0.0.0 --port "$PORT" \
        --api-key "$KEY" --max-model-len 8192 > "$LOG" 2>&1 &
    pid=$!
    echo "starting $MODEL (first run downloads ~16 GB, a few minutes), log: $LOG"
    for _ in $(seq 180); do   # 15 min
        up && break
        kill -0 "$pid" 2>/dev/null || { tail -30 "$LOG"; echo "server died, see above"; exit 1; }
        sleep 5
    done
    up || { tail -30 "$LOG"; echo "not ready after 15 min"; exit 1; }
fi

cat <<EOF

READY on port $PORT. On the Pi:
  1. Raspberry/config.json:  "brev_url": "http://$(curl -s https://api.ipify.org || echo BREV-HOST):$PORT"   (IP of this box; check in the Brev console)
  2. export BREV_KEY=$KEY
  3. python vision.py ask p.jpg
If the Pi cannot connect, expose TCP port $PORT for this instance in the Brev console.
EOF
