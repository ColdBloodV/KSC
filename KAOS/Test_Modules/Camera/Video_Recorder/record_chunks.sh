#!/usr/bin/env bash
set -u

# KAOS Pi Zero flight video recorder.
# Records fixed-length H.264 chunks outside the repo and uses the ACT LED
# as a simple visual health indicator.

MEDIA_OWNER="${MEDIA_OWNER:-kaos}"
MEDIA_HOME="${MEDIA_HOME:-/home/$MEDIA_OWNER}"
OUT_ROOT="${OUT_ROOT:-$MEDIA_HOME/kaos_media/video}"
LOG_ROOT="${LOG_ROOT:-$MEDIA_HOME/kaos_media/logs}"

CHUNK_MS="${CHUNK_MS:-60000}"
MAX_CHUNKS="${MAX_CHUNKS:-0}"

WIDTH="${WIDTH:-1920}"
HEIGHT="${HEIGHT:-1080}"
FPS="${FPS:-30}"
BITRATE="${BITRATE:-8000000}"
LOW_SPACE_KB="${LOW_SPACE_KB:-1048576}"

LED_PATH=""
LED_PID=""
PREVIOUS_LED_TRIGGER=""
CLEANED_UP=0
CURRENT_TMP_FILE=""

find_led() {
  for candidate in /sys/class/leds/ACT /sys/class/leds/led0; do
    if [ -d "$candidate" ]; then
      LED_PATH="$candidate"
      return 0
    fi
  done
  return 1
}

led_init() {
  find_led || return 0

  if [ -r "$LED_PATH/trigger" ]; then
    PREVIOUS_LED_TRIGGER="$(cat "$LED_PATH/trigger" 2>/dev/null | sed -n 's/.*\[\([^]]*\)\].*/\1/p')"
  fi

  echo none > "$LED_PATH/trigger" 2>/dev/null || true
  echo 0 > "$LED_PATH/brightness" 2>/dev/null || true
}

led_restore() {
  [ -n "$LED_PID" ] && kill "$LED_PID" 2>/dev/null || true
  [ -n "$LED_PID" ] && wait "$LED_PID" 2>/dev/null || true
  LED_PID=""

  if [ -n "$LED_PATH" ]; then
    echo 0 > "$LED_PATH/brightness" 2>/dev/null || true
    if [ -n "$PREVIOUS_LED_TRIGGER" ]; then
      echo "$PREVIOUS_LED_TRIGGER" > "$LED_PATH/trigger" 2>/dev/null || true
    fi
  fi
}

led_on() {
  [ -n "$LED_PATH" ] && echo 1 > "$LED_PATH/brightness" 2>/dev/null || true
}

led_off() {
  [ -n "$LED_PATH" ] && echo 0 > "$LED_PATH/brightness" 2>/dev/null || true
}

led_blink() {
  local count="$1"
  local on_time="${2:-0.1}"
  local off_time="${3:-0.1}"

  for ((b=0; b<count; b++)); do
    led_on
    sleep "$on_time"
    led_off
    sleep "$off_time"
  done
}

led_recording_pattern() {
  while true; do
    led_on
    sleep 0.08
    led_off
    sleep 0.92
  done
}

start_recording_led() {
  led_recording_pattern &
  LED_PID="$!"
}

stop_recording_led() {
  [ -n "$LED_PID" ] && kill "$LED_PID" 2>/dev/null || true
  [ -n "$LED_PID" ] && wait "$LED_PID" 2>/dev/null || true
  LED_PID=""
  led_off
}

log_line() {
  echo "$*" | tee -a "$LOG_FILE"
}

path_owner_fix() {
  local path="$1"
  if id "$MEDIA_OWNER" >/dev/null 2>&1; then
    chown "$MEDIA_OWNER:$MEDIA_OWNER" "$path" 2>/dev/null || true
  fi
}

cleanup() {
  if [ "$CLEANED_UP" -eq 1 ]; then
    return
  fi
  CLEANED_UP=1

  stop_recording_led
  if [ -n "$CURRENT_TMP_FILE" ] && [ -f "$CURRENT_TMP_FILE" ]; then
    rm -f "$CURRENT_TMP_FILE"
  fi
  log_line "KAOS camera recorder stopping: $(date -u)"
  sync
  path_owner_fix "$LOG_FILE"
  led_restore
}

if ! command -v rpicam-vid >/dev/null 2>&1; then
  echo "ERROR: rpicam-vid is not installed or not on PATH" >&2
  exit 127
fi

HOST="$(hostname)"
SESSION="$(date -u +%Y%m%dT%H%M%SZ)_${HOST}"
OUT_DIR="$OUT_ROOT/$SESSION"
LOG_FILE="$LOG_ROOT/camera_${SESSION}.log"

mkdir -p "$OUT_DIR" "$LOG_ROOT"
path_owner_fix "$MEDIA_HOME/kaos_media"
path_owner_fix "$OUT_ROOT"
path_owner_fix "$LOG_ROOT"
path_owner_fix "$OUT_DIR"

trap cleanup EXIT
trap 'cleanup; exit 0' INT TERM

led_init
led_blink 5 0.08 0.08

log_line "KAOS camera recorder started: $(date -u)"
log_line "Host: $HOST"
log_line "Output: $OUT_DIR"
if [ "$MAX_CHUNKS" -eq 0 ]; then
  CHUNK_LIMIT_LABEL="unlimited"
else
  CHUNK_LIMIT_LABEL="$MAX_CHUNKS"
fi

log_line "Settings: ${WIDTH}x${HEIGHT} ${FPS}fps bitrate=${BITRATE} chunk_ms=${CHUNK_MS} max_chunks=${CHUNK_LIMIT_LABEL}"

i=0
while [ "$MAX_CHUNKS" -eq 0 ] || [ "$i" -lt "$MAX_CHUNKS" ]; do
  free_kb="$(df -Pk "$OUT_ROOT" | awk 'NR==2 {print $4}')"

  if [ -n "$free_kb" ] && [ "$free_kb" -lt "$LOW_SPACE_KB" ]; then
    log_line "LOW SPACE: less than ${LOW_SPACE_KB}KB free, stopping recorder"
    led_blink 20 0.05 0.05
    sync
    exit 2
  fi

  TS="$(date -u +%Y%m%dT%H%M%SZ)"
  BASE="video_${TS}_$(printf "%06d" "$i")"
  TMP_FILE="$OUT_DIR/${BASE}.partial.h264"
  FINAL_FILE="$OUT_DIR/${BASE}.h264"
  CURRENT_TMP_FILE="$TMP_FILE"

  log_line "[$(date -u)] Starting chunk $i: $FINAL_FILE"

  start_recording_led

  if rpicam-vid \
    --nopreview \
    -t "$CHUNK_MS" \
    --width "$WIDTH" \
    --height "$HEIGHT" \
    --framerate "$FPS" \
    --bitrate "$BITRATE" \
    --codec h264 \
    --inline \
    -o "$TMP_FILE" >> "$LOG_FILE" 2>&1; then

    stop_recording_led
    mv "$TMP_FILE" "$FINAL_FILE"
    CURRENT_TMP_FILE=""
    sync
    path_owner_fix "$FINAL_FILE"
    path_owner_fix "$LOG_FILE"

    SIZE="$(ls -lh "$FINAL_FILE" | awk '{print $5}')"
    TEMP="$(vcgencmd measure_temp 2>/dev/null || true)"
    THROTTLE="$(vcgencmd get_throttled 2>/dev/null || true)"
    DISK="$(df -h "$OUT_ROOT" | awk 'NR==2 {print $4 " free"}')"

    log_line "[$(date -u)] Finished chunk $i size=$SIZE temp=$TEMP throttle=$THROTTLE disk=$DISK"
    led_blink 3 0.06 0.08
  else
    stop_recording_led
    log_line "[$(date -u)] ERROR: chunk $i failed"
    rm -f "$TMP_FILE"
    CURRENT_TMP_FILE=""
    led_blink 10 0.05 0.05
    sync
    sleep 3
  fi

  i=$((i + 1))
done

log_line "KAOS camera recorder completed all chunks: $(date -u)"
sync
