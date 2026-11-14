#!/usr/bin/env bash
# No ROS/Python pip environment is required; Ubuntu Desktop supplies Python 3.
set -euo pipefail
HERE="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd -P)"
exec /usr/bin/python3 "$HERE/student_install.py" "$@"
