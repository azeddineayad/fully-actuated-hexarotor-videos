# Fully actuated hexarotor with tilted rotors: scenario videos

Videos of the simulation scenarios of the paper *"A Fully Actuated Hexarotor with Tilted Rotors: Modelling,
Construction in the MRS UAV System, Control and Autonomous Flight Scenarios"* (A. AYAD). The vehicle is an F680
hexarotor whose six fixed rotors are tilted alternately by 28° about their arms, so that it translates with its body
level. In every video the drone is flown by its **Pixhawk 6C flight code** (ArduPilot ArduCopter, 6DoF frame) in
software in the loop, against the physics of the vehicle, and shown in the MRS UAV System Gazebo world. Only the
flight is shown (no commands); sped-up parts are labelled in the video.

| Scenario | Paper | Video | What it shows |
|---|---|---|---|
| 3. B-spline trajectory | Section 5.2, Fig. 8 | [scenario3_bspline.webm](videos/scenario3_bspline.webm) | 40.7 m cubic B-spline, heading along the path, body level (live plots: error, roll/pitch, yaw, rotor forces) |
| 4. External disturbances | Section 5.3, Fig. 9 | [scenario4_disturbance.webm](videos/scenario4_disturbance.webm) | circle of Flores et al. (R 4 m, 2 m/s) under their disturbances scaled by 0.05 |
| 5. Suspended pendulum | Section 5.4, Fig. 10 | [scenario5_pendulum.webm](videos/scenario5_pendulum.webm) | 0.5 m rod with 0.25 kg under the body, B-spline from and to the ground |
| 6. Unknown walls | Section 5.5, Fig. 11 | [scenario6_walls.webm](videos/scenario6_walls.webm) | the walls are seen only with the onboard RGB-D camera (top left), map and path (bottom right) |
| 7. Labyrinth | Section 5.6, Fig. 13 | [scenario7_labyrinth.webm](videos/scenario7_labyrinth.webm) | escape from a 7×7 maze with the depth camera: dead ends, re-planning; live rotor forces, Euler angles, velocity |
| 8. Cleaning of solar panels | Section 5.7, Figs. 13–14 | [scenario8_solar_cleaning.webm](videos/scenario8_solar_cleaning.webm) | two panels tilted 10°, body pitched 10° so that the nozzle faces the glass, water spray during 8 forward/backward passes at 0.6 m from the glass |

Scenarios 1 and 2 (validation of the model, circles at level attitude) were flown in the MRS simulation with the
simulation controller; they have no video here (Fig. 7 of the paper).

All the videos are simulations: the real aircraft has not flown yet. Each video is one flight; the figures of the
paper come from the same flights.

`tools/` contains the scripts that recorded and cut the videos.

## Second paper: optimal tilt and dihedral angles of the rotors

Videos of the paper *"Optimal Tilt and Dihedral Angles of a Fixed-Tilt Fully Actuated Hexarotor: Exact Wrench Indices,
Rotor-Failure Tolerance and Flight-Code Validation"* (A. AYAD). Each rotor of the F680 gets a tilt about its arm and a
dihedral along it; the design minimizing the hover power under four requirements is a 28° tilt with a 9.5° outward
dihedral. Each flight video shows the **reference** design (28°, no dihedral) and then the **optimum** (28°, −9.5°),
flown by the Pixhawk 6C flight code (ArduPilot) in software in the loop, with the live plots of the flight code; it
ends with the figure of the paper made from these two flights. Flights sped up ×3 (labelled).

| Scenario | Paper | Video | What it shows |
|---|---|---|---|
| 0. Optimization | Section 5.3, Fig. 4 | [design_optimization.webm](videos/tilt_dihedral_design/design_optimization.webm) | NSGA-II population at each of the 100 generations, in the design plane and against the exact Pareto front of the map, with the convergence (IGD) |
| 1. Nominal flight | Section 7.2, Fig. 5 | [design_circle.webm](videos/tilt_dihedral_design/design_circle.webm) | two circles of radius 4 m at 2 m/s with the body level; tracking and hover power of both designs |
| 2. Attitude held in hover | Section 7.3, Fig. 6a | [design_attitude_hold.webm](videos/tilt_dihedral_design/design_attitude_hold.webm) | body held at 12° in the weakest direction; the reference drops below the 1 N rotor margin, the optimum keeps it |
| 3. Gusts | Section 7.4, Fig. 6b | [design_gust.webm](videos/tilt_dihedral_design/design_gust.webm) | three 4 N gusts at level attitude; identical for both designs (limited by the position loop) |
| 4. Rotor failure | Section 7.5, Fig. 7 | [design_rotor_failure.webm](videos/tilt_dihedral_design/design_rotor_failure.webm) | rotor 1 cut in hover at 5 m, five-rotor mixer (research copy of ArduPilot, simulation only), hover tilted 23°, landing on five rotors |

All these videos are simulations. `tools/tilt_dihedral_design/` contains the scripts that recorded and assembled them.
