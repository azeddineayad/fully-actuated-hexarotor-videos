#!/usr/bin/env bash
# Cut one scenario video from a raw screen recording (1366 x 768): keep only the flight (start..end s), crop away
# the desktop bars, black out the given terminal rectangles, speed up, label, encode WebM (VP9).
#   cut_video.sh <raw.webm> <out.webm> <start_s> <end_s> <speed> "<label>" ["x:y:w:h" ...]   (rectangles in crop pixels)
set -euo pipefail
F=${FFMPEG:-$HOME/tools/bin/ffmpeg}
raw=$1; out=$2; t0=$3; t1=$4; speed=$5; label=$6; shift 6
font=$(fc-match -f '%{file}' 'DejaVu Sans:bold')
vf="crop=1300:736:66:32"
for r in "$@"; do
  IFS=: read -r x y w h <<< "$r"
  vf="$vf,drawbox=x=$x:y=$y:w=$w:h=$h:color=black:t=fill"
done
vf="$vf,setpts=(PTS-STARTPTS)/$speed,fps=25"
vf="$vf,drawtext=fontfile=$font:text='$label':fontsize=20:fontcolor=white:box=1:boxcolor=black@0.7:boxborderw=6:x=(w-tw)/2:y=h-34"
"$F" -v error -y -ss "$t0" -to "$t1" -i "$raw" -an -vf "$vf" \
  -c:v libvpx-vp9 -b:v 0 -crf 36 -deadline good -cpu-used 4 -row-mt 1 "$out"
ls -la "$out"
