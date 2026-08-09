#!/usr/bin/env bash
# Runs both suites. Kept inside the repo on purpose: an earlier version of
# these harnesses lived in /tmp and was lost when the machine restarted,
# taking the only proof the firmware worked with it.
set -u
cd "$(dirname "$0")"
rc=0

echo "──────── firmware (C++, compiled against tests/stub) ────────"
if g++ -std=c++17 -w -Istub -o firmware_check firmware_check.cpp; then
  ./firmware_check || rc=1
else
  echo "  firmware_check FAILED TO COMPILE"; rc=1
fi

echo
echo "──────── GUI logic (Python, headless tk stub) ────────"
python3 python_check.py || rc=1

echo
[ $rc -eq 0 ] && echo "════ EVERYTHING GREEN ════" || echo "════ SOMETHING FAILED ════"
exit $rc
