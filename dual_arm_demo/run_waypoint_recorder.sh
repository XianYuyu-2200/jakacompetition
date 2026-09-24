#!/usr/bin/env bash
set -euo pipefail

ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$ROOT/robot_roles.env"

OUTPUT="${1:-$ROOT/coffee_waypoints.yaml}"

echo "Starting READ-ONLY waypoint recorder"
echo "Robot: $TRACKING_IP"
echo "Output: $OUTPUT"
echo "Move the robot only with JAKA manual/jog/drag controls."

exec "$ROOT/waypoint_recorder" \
  --robot-ip "$TRACKING_IP" \
  --output "$OUTPUT"
