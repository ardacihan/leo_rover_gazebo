#!/usr/bin/env bash
# Print per-robot path length and x/y spread from a traj.csv (arg 1).
# Used to check whether leo2 escapes the corridor (|y| grows) after the
# own-map Nav2 fix.
f="${1:-/ros2_ws/reports/collab_fixtest/office_coordinated/traj.csv}"
[ -f "$f" ] || { echo "no traj yet: $f"; exit 0; }
awk -F, 'NR>1{
  n[$2]++;
  if(n[$2]>1){d[$2]+=sqrt(($3-px[$2])^2+($4-py[$2])^2)}
  px[$2]=$3; py[$2]=$4;
  if(minx[$2]==""||$3<minx[$2])minx[$2]=$3; if($3>maxx[$2])maxx[$2]=$3;
  if(miny[$2]==""||$4<miny[$2])miny[$2]=$4; if($4>maxy[$2])maxy[$2]=$4;
}
END{for(r in n) printf "%s: pathlen=%.1fm x=[%.1f,%.1f] y=[%.1f,%.1f] (yspan=%.1f) n=%d\n",
    r, d[r], minx[r], maxx[r], miny[r], maxy[r], maxy[r]-miny[r], n[r]}' "$f"
