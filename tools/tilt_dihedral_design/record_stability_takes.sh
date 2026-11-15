#!/bin/bash
# v5, design paper Scenario 5 (stable and unstable rotor angles): screen-record the circle scenario for designs of
# decreasing tilt (Gazebo + live plots, commands hidden in tmux by ~/fully_actuated_hexarotor_videos/tools/record_flight.py).
# SITL only.  The flight logs go to analysis/results/stability_runs (the figure of the paper is made from these flights).
# Ends with the optimum design selected again.  Run it detached, under an idle inhibitor:
#   setsid -w record_stability_takes.sh [designs...] < /dev/null > log 2>&1      default: reference tilt20 tilt15 tilt10 tilt5
P=$HOME/fully_actuated_hexacopter_f680_v5
OUT=$P/analysis/results/video_stability
mkdir -p "$OUT" "$P/analysis/results/stability_runs"
designs=("$@")
[ $# -eq 0 ] && designs=(reference tilt20 tilt15 tilt10 tilt5)
for d in "${designs[@]}"; do
  "$P/select_design.sh" "$d" > "$OUT/select_$d.log" 2>&1 || { echo "select_design $d failed"; exit 1; }
  for try in 1 2 3; do
    echo "=== $(date +%T) circle, $d, take $try"
    gnome-session-inhibit --inhibit idle --reason "recording the v5 stability videos" \
      python3 "$HOME/fully_actuated_hexarotor_videos/tools/record_flight.py" --out "$OUT/circle_${d}_raw.webm" \
        --ws "$HOME/hexacopter_ws_f680_v5" --stop-sim "$P/analysis/stop_simulation.sh" \
        --flight "cd $P && SCENARIO_DIR=results/stability_runs ./fly_pixhawk_sim.sh --scenario circle --research-fw" \
        --start "Mission: arm" --stop "disarmed, landed|still armed after|did not end with a landing" --track \
        --layout "Gazebo Sim=900x700+0+32" "fa_hexacopter live=460x700+904+32" > "$OUT/take_circle_$d.log" 2>&1
    tail -2 "$OUT/take_circle_$d.log"
    grep -q "TAKE OK" "$OUT/take_circle_$d.log" && break
  done
done
"$P/select_design.sh" optimum > /dev/null 2>&1 && echo "optimum design selected again"
