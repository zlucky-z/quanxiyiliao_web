#!/bin/bash
set -euo pipefail

log_warn() {
  echo "[run-server] warn: $*" >&2
}

ensure_gpio_output() {
  local pin="$1"
  local pinmux_name="$2"

  if command -v cvi_pinmux >/dev/null 2>&1; then
    cvi_pinmux -w "$pinmux_name" || log_warn "cvi_pinmux setup failed for ${pinmux_name}"
  else
    log_warn "cvi_pinmux not found, skip pinmux for gpio${pin}"
  fi

  if [ ! -d "/sys/class/gpio/gpio${pin}" ]; then
    if ! printf "%s\n" "$pin" | tee /sys/class/gpio/export >/dev/null; then
      log_warn "failed to export gpio${pin}"
      return 1
    fi
  fi

  if ! printf "out\n" | tee "/sys/class/gpio/gpio${pin}/direction" >/dev/null; then
    log_warn "failed to set gpio${pin} direction to out"
    return 1
  fi

  if ! chmod 666 "/sys/class/gpio/gpio${pin}/direction" "/sys/class/gpio/gpio${pin}/value"; then
    log_warn "failed to chmod gpio${pin} direction/value"
  fi
}

is_cef_server_running() {
  local pid exe
  while read -r pid; do
    [ -z "${pid}" ] && continue
    exe=$(readlink -f "/proc/${pid}/exe" 2>/dev/null || true)
    case "${exe}" in
      "/data/lintech/celectronicfence/server"|"/data/lintech/celectronicfence/server (deleted)")
        return 0
        ;;
    esac
  done < <(pgrep -x server 2>/dev/null || true)
  return 1
}

echo "[run-server] start mediamtx..."
if pgrep -x mediamtx >/dev/null 2>&1; then
  echo "[run-server] mediamtx already running"
else
  cd /data/lintech
  nohup /data/lintech/mediamtx /data/lintech/mediamtx.yml > /data/lintech/mediamtx.log 2>&1 &
  echo "[run-server] mediamtx pid=$!"
fi

echo "[run-server] start celectronicfence server..."
if is_cef_server_running; then
  echo "[run-server] server already running"
else
  cd /data/lintech/celectronicfence
  nohup script -q -c '/data/lintech/celectronicfence/server' /data/lintech/celectronicfence/server.log >/dev/null 2>&1 &
  echo "[run-server] server pid=$!"
fi

echo "[run-server] init gpio (optional)..."
if [ "$(id -u)" -eq 0 ]; then
  ensure_gpio_output 429 "PWM2/GPIO77" || true
  ensure_gpio_output 430 "PWM3/GPIO78" || true
else
  echo "[run-server] skip gpio init (not root)"
fi

echo "[run-server] done"
