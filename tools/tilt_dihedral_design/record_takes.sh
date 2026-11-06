#!/bin/bash
# v5 scenario videos: screen-record a design scenario for each design (Gazebo + live plots, commands hidden in tmux by
# ~/fully_actuated_hexarotor_videos/tools/record_flight.py).  The flight logs go to analysis/results/video_design_runs
# (the figures of the paper are made from these same flights).  Ends with the optimum design selected.
# Run it detached (inside a terminal job, gnome-session-inhibit was stopped by SIGTTIN):
#   setsid -w record_takes.sh <scenario> [designs...] < /dev/null > log 2>&1
#   scenarios: circle | attitude_hold | gust | rotor_failure      designs: default reference optimum
P=$HOME/fully_actuated_hexacopter_f680_v5
OUT=$P/analysis/results/video_design
mkdir -p "$OUT"
sc=${1:?scenario}; shift
designs=("$@")
[ $# -eq 0 ] && designs=(reference optimum)
case "$sc" in
  attitude_hold) extra="--azimuth 5 --tilt 12" ;;
  *) extra="" ;;
esac
for d in "${designs[@]}"; do
  "$P/select_design.sh" "$d" > "$OUT/select_$d.log" 2>&1 || { echo "select_design $d failed"; exit 1; }
  for try in 1 2 3; do
    echo "=== $sc, $d, take $try"
    gnome-session-inhibit --inhibit idle --reason "recording the v5 design videos" \
      python3 "$HOME/fully_actuated_hexarotor_videos/tools/record_flight.py" --out "$OUT/${sc}_${d}_raw.webm" \
        --ws "$HOME/hexacopter_ws_f680_v5" --stop-sim "$P/analysis/stop_simulation.sh" \
        --flight "cd $P && SCENARIO_DIR=results/video_design_runs ./fly_pixhawk_sim.sh --scenario $sc --research-fw $extra" \
        --start "Mission: arm" --stop "disarmed, landed" --track \
        --layout "Gazebo Sim=900x700+0+32" "fa_hexacopter live=460x700+904+32" > "$OUT/take_${sc}_$d.log" 2>&1
    tail -2 "$OUT/take_${sc}_$d.log"
    grep -q "TAKE OK" "$OUT/take_${sc}_$d.log" && break
  done
done
"$P/select_design.sh" optimum > /dev/null 2>&1 && echo "optimum design selected again"
