#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$ROOT/robot_roles.env"

TOKEN="${1:-}"
DURATION="${2:-20}"
if [[ "$TOKEN" != "START_J1" ]]; then
  echo "Usage: $0 START_J1 [duration_sec]" >&2
  exit 64
fi

rm -f "$IPC_SOCKET"
cleanup() {
  if [[ -n "${OPERATOR_PID:-}" ]]; then
    kill "$OPERATOR_PID" 2>/dev/null || true
    wait "$OPERATOR_PID" 2>/dev/null || true
  fi
  if [[ -n "${FOLLOWER_PID:-}" ]]; then
    kill "$FOLLOWER_PID" 2>/dev/null || true
    wait "$FOLLOWER_PID" 2>/dev/null || true
  fi
  rm -f "$IPC_SOCKET"
}
trap cleanup EXIT INT TERM

echo "J1 follow test: $OPERATOR_IP -> $TRACKING_IP"
echo "J1 relative motion is 1:1; J2-J6 remain at startup positions"
echo "No automatic robot power-on or robot enable"

"$ROOT/j1_follow_test" \
  "$TRACKING_IP" "$IPC_SOCKET" START_J1 "$DURATION" &
FOLLOWER_PID=$!

for _ in $(seq 1 200); do
  [[ -S "$IPC_SOCKET" ]] && break
  if ! kill -0 "$FOLLOWER_PID" 2>/dev/null; then
    echo "J1 follower exited before creating IPC socket" >&2
    exit 1
  fi
  sleep 0.01
done

if [[ ! -S "$IPC_SOCKET" ]]; then
  echo "IPC socket was not created" >&2
  exit 1
fi

"$ROOT/operator_joint_publisher" \
  "$OPERATOR_IP" "$IPC_SOCKET" "0" &
OPERATOR_PID=$!

set +e
wait "$FOLLOWER_PID"
FOLLOWER_RC=$?
set -e
kill "$OPERATOR_PID" 2>/dev/null || true
set +e
wait "$OPERATOR_PID"
OPERATOR_RC=$?
set -e
if [[ "$OPERATOR_RC" -eq 143 ]]; then
  OPERATOR_RC=0
fi

echo "operator exit code: $OPERATOR_RC"
echo "J1 follower exit code: $FOLLOWER_RC"
exit "$(( OPERATOR_RC != 0 || FOLLOWER_RC != 0 ))"
