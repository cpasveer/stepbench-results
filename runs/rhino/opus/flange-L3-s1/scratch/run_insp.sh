#!/bin/sh
# Run submission.py inside the open Rhino and wait until it has finished (rhinocode itself returns at once).
HERE="$(cd "$(dirname "$0")/.." && pwd)"
RHINOCODE="${RHINOCODE:-/Applications/Rhino 9.app/Contents/Resources/bin/rhinocode}"
rm -f "$HERE/submission.log"
OUT="$("$RHINOCODE" script "$HERE/scratch/insp.py" 2>&1)"
if printf '%s' "$OUT" | grep -qiE "timeout|failed to connect|error"; then
  echo "Rhino could not be reached: $(printf '%s' "$OUT" | head -2 | sed "s#$HERE#.#g")"
  echo "This is a problem of the test machine, not of your script. Wait a minute and try again; if it repeats, stop and report it."
  exit 2
fi
i=0
while [ ! -s "$HERE/submission.log" ]; do
  i=$((i+1)); [ $i -gt 600 ] && { echo "no result after 10 minutes"; exit 3; }
  sleep 1
done
cat "$HERE/submission.log"
grep -q "^ok:" "$HERE/submission.log"
