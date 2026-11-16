# Custom controller: LQR with integral action

A 12-state discrete LQR with integral action (LQI) runs inside PX4 SITL as an alternative to PX4's cascaded PID controllers. It is selected with the profile **LQI** in `Main.mlx`'s `PX4TuneChoice` drop-down, which sets `CUSTOM_CONTROLLER = 1` (0 = normal PX4 flight), or with `run_mission(plan, Gains="LQI")`. `custom_controller_parameters("lqr")` gives the same controller without integral action.

## How it runs in PX4 (offboard actuator mode, no firmware change)

- **Switch:** `PX4_Receive.slx` has a subsystem **"Custom Controller (LQR)"**, enabled by `CUSTOM_CONTROLLER && ~TUNING_ON`. The mission publishers are enabled by `~TUNING_ON && ~CUSTOM_CONTROLLER`, so only one branch is active at a time.
- **Inputs and controller:** the subsystem reads PX4's state estimate (`vehicle_local_position`, `vehicle_attitude`, `vehicle_angular_velocity`) and runs `lqr_hover_control.m` at 100 Hz against the `FlightPlan` reference (the same reference the PX4 mission gets).
- **Outputs:** it publishes `actuator_motors` and `offboard_control_mode` with only `actuator = true`. In PX4 v1.14.3, OFFBOARD with `actuator` set clears every controller flag: `mc_rate_control`, `mc_att_control` and `mc_pos_control` publish nothing and the custom controller is the only source of motor commands.
- **Arming:** the time-triggered commands of `PX4_Receive` set OFFBOARD and arm, as for PX4 missions.
- **Motor scaling:** the controller works in normalised rotor speed m (the Simscape input). PX4 applies `THR_MDL_FAC = 1` (output = √thrust), so the subsystem publishes m². The hover command m₀ is the linearisation point of the design.
- **Gains in this repository:** the stored gains were designed for an earlier vehicle configuration, not for the unit drone; `check_gain_validity` warns when they are used. Redesign them for the current setup as described under Design.
- **Auto-disarm:** with no PX4 thrust setpoint the land detector can report "landed" in a steady hover, so the subsystem writes `COM_DISARM_LAND = -1` once at t = 1.5 s. SITL only; not for a real vehicle.
- **Tuning runners** set `CUSTOM_CONTROLLER = false`, so tuning is unaffected.

## Control law (`lqr_hover_control.m`)

m = m₀ − K·x − Kᵢ·z, with
- **x** = [N E D, vN vE vD, yaw pitch roll, p q r] − reference. Position and velocity errors are rotated into the yaw-aligned frame and limited to ±1 m and ±2 m/s; the yaw error to ±0.5 rad. With `VelocityFeedforward` the reference velocity and yaw rate are part of the reference.
- **z** = integrals of the N, E, D and yaw errors. They start after the takeoff climb, freeze while a motor command saturates, are capped (at most 0.06 of motor command each), and are kept in North-East-Down and rotated into the yaw frame when applied, so they stay correct while the heading turns along the path.

## Design (`linearize_drone`, `design_custom_controller`)

```matlab
[~,~,~,~,result] = linearize_drone;                       % hover linearisation (no PX4 needed)
D = design_custom_controller(result);                     % both designs and their checks (D.Checks)
design_custom_controller(result, WriteParameters=true);   % writes custom_controller_parameters.m
```

- **Linearisation:** `linearize_drone` builds private analysis copies of the plant (the originals are never saved), starts at hover 1 m above ground and linearises after a 0.1 s snapshot. The 12-state model treats the actuators as quasi-steady and leaves out the four propeller-angle states, which are pure integrators that nothing measures (they made the old full-model design fail). `result.FullModel` keeps the full linearisation with motor dynamics for the checks.
- **Weights:** discrete design at Ts = 0.01 s (the `PX4_Receive` rate); Bryson Q with allowed sizes 0.5 m, 1 m/s, 10°, 30°/s; R = 1000·I. Integral weights 0.5 m·s and 0.5 rad·s.
- **Checks on the full plant with motor dynamics** (0 / 1 / 2 extra samples of delay at the motors):

  | | Stable | Phase margin at the motors | Slowest time constant |
  |---|---|---|---|
  | LQR | yes / yes / yes | 39° / 38° / 27° | 1.2 s |
  | LQI | yes / yes / yes | 34° / 33° / 25° | 2.9 s |

- **Constant disturbances (linear model):** a hover-thrust error of 0.01, a 0.2 m/s² horizontal push and a yaw-torque bias leave permanent offsets of 32 cm, 12 cm and 8° with the LQR; the LQI removes them within about 5 s.
- Recompute the linearisation and the design after any change of vehicle, payload, motor, propeller or plant.

## Results (SITL, figure-8 4 m × 2 m in 20 s, heading along the path, same reference as PX4)

| | LQR | LQI | PX4 PID, RMSE-tuned gains |
|---|---:|---:|---:|
| Tracking error RMS | 28.2 cm | 18.9 cm | 12.1 cm |
| Distance from the path RMS | 16.7 cm | 14.5 cm | 5.4 cm |
| Altitude error RMS | 8.0 cm | 4.5 cm | 4.5 cm |
| Heading error RMS | 7.8° | 8.3° | 1.2° |
| Motor-command rate RMS | 0.35 /s | 0.44 /s | 1.9 /s |

## Limits

- Hover linearisation: large manoeuvres leave the linear region (the error limits keep the commands sensible).
- Yaw is the slowest axis; with the heading following the path the heading error is about 8° RMS.
- Validated in SITL only (first flight 11 November 2026).
- The MathWorks Hardware Setup option that removes PX4's controllers ("Design Flight Controller in Simulink") would also work but needs a firmware rebuild; it is not used.
