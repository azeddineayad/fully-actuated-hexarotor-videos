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
