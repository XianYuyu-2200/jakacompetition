#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$ROOT/robot_roles.env"

DURATION="${1:-0}"
CSV_PATH="${2:-}"
rm -f "$IPC_SOCKET"

cleanup() {
  if [[ -n "${OPERATOR_PID:-}" ]]; then
    kill "$OPERATOR_PID" 2>/dev/null || true
    wait "$OPERATOR_PID" 2>/dev/null || true
  fi
  if [[ -n "${TRACKING_PID:-}" ]]; then
    kill "$TRACKING_PID" 2>/dev/null || true
    wait "$TRACKING_PID" 2>/dev/null || true
  fi
  rm -f "$IPC_SOCKET"
}
trap cleanup EXIT INT TERM

echo "operator arm: $OPERATOR_IP"
echo "tracking arm: $TRACKING_IP"
echo "data flow: $OPERATOR_IP -> $TRACKING_IP"
echo "dry-run only: no robot motion commands"

TRACKING_ARGS=("$TRACKING_IP" "$IPC_SOCKET" "$DURATION")
if [[ -n "$CSV_PATH" ]]; then
  TRACKING_ARGS+=("$CSV_PATH")
fi
"$ROOT/tracking_arm_dry_run" "${TRACKING_ARGS[@]}" &
TRACKING_PID=$!

for _ in $(seq 1 200); do
  [[ -S "$IPC_SOCKET" ]] && break
  if ! kill -0 "$TRACKING_PID" 2>/dev/null; then
    echo "tracking dry-run exited before creating IPC socket" >&2
    exit 1
  fi
  sleep 0.01
done

if [[ ! -S "$IPC_SOCKET" ]]; then
  echo "IPC socket was not created" >&2
  exit 1
fi

"$ROOT/operator_joint_publisher" \
  "$OPERATOR_IP" "$IPC_SOCKET" "$DURATION" &
OPERATOR_PID=$!

wait "$OPERATOR_PID"
OPERATOR_RC=$?
wait "$TRACKING_PID"
TRACKING_RC=$?

echo "operator exit code: $OPERATOR_RC"
echo "tracking exit code: $TRACKING_RC"
exit "$(( OPERATOR_RC != 0 || TRACKING_RC != 0 ))"
