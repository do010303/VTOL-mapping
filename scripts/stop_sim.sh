#!/bin/bash
# Stops what this example started (the PIDs its launchers recorded in run/pids); nothing else.
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PIDS="$HERE/run/pids"
[ -s "$PIDS" ] || { echo "nothing recorded in $PIDS"; exit 0; }
pids=$(for p in $(cat "$PIDS"); do kill -0 "$p" 2>/dev/null && echo "$p"; done)
[ -n "$pids" ] && kill $pids 2>/dev/null && sleep 3 && kill -KILL $pids 2>/dev/null
: > "$PIDS"
echo "stopped: ${pids:-none}"
