# Results, 16 November 2026

This page continues [results_20261112](../results_20261112/README.md). Every figure is given twice:
- `_CB`: colour-blind safe (viridis colours plus line styles);
- `_RG`: standard red/green/blue, not colour-blind safe.

Each figure is a PDF (vector, for LaTeX) and a PNG.

All flights are PX4 v1.14.3 SITL against the Simscape plant, after the magnetometer fix. They are scored on the plant's physical truth, not on PX4's own estimate.

These results were obtained with an earlier vehicle configuration, not with the unit drone this repository now models. They show what the tools do; the current parameters do not reproduce these numbers.

## 1. Automatic tuning with the RMSE score

**The run:**
- Started from PX4 Default with `run_px4_guide_tuning("all", Method="rmse")`.
- 69 flights in 15.1 h wall time. One flight hit the known PX4 startup stall and was repeated; no flight was scored from a failed run.
- Order: rates → attitude → velocity → position, then a final check of every group.

**The score per loop** (lower is better; J = RMSE/A + 0.30·max|e|/A + 0.25·overshoot on the step tests; mean of 3 baseline flights):

| Loop | Default | Tuned | Noise margin |
|---|---:|---:|---:|
| Roll rate | 1.56 | 0.19 | 0.07 |
| Pitch rate | 1.55 | 0.20 | 0.09 |
| Yaw rate | 0.67 | 0.28 | 0.02 |
| Roll attitude | 0.38 | 0.30 | 0.03 |
| Pitch attitude | 0.37 | 0.37 (no significant change) | 0.04 |
| Yaw attitude | 0.52 | 0.45 | 0.01 |
| Horizontal velocity | 0.60 | 0.50 | 0.003 |
| Vertical velocity | 0.40 | 0.26 | 0.007 |
| Horizontal position | 0.71 | 0.65 | 0.009 |
| Vertical position | 0.72 | 0.65 | 0.004 |

**The tuned gains** (relative to Default; all others unchanged):

| Gain | Default | Tuned | |
|---|---:|---:|---:|
| `MC_ROLLRATE_K`, `MC_PITCHRATE_K`, `MC_YAWRATE_K` | 1 | 0.654 | 65 % |
| `MC_ROLL_P` | 6.5 | 10.99 | 169 % |
| `MC_YAW_P` | 2.8 | 5.0 | 179 %, PX4 maximum |
| `MPC_XY_VEL_P_ACC` | 1.8 | 5.0 | 278 %, PX4 maximum |
| `MPC_Z_VEL_P_ACC` | 4 | 15 | 375 %, PX4 maximum |
| `MPC_XY_P` | 0.95 | 1.61 | 169 % |
| `MPC_Z_P` | 1 | 1.5 | 150 % |

**Findings:**
- **Default rate gains oscillate on this airframe.** One step ×0.77 on K removes most of it (score 1.56 → 0.19).
- **The robustness probe did its job.** The first ×1.3 probe of the rate loops oscillated, so all three rate loops were backed off ×0.85 (K = 0.77 × 0.85 = 0.654); the repeated probe passed. Every later group passed its probe directly, so every loop has a gain margin of at least 1.3.
- **Three gains stopped at PX4's upper parameter limit.** The search would have gone further. On the real drone, gains this high would be limited by sensor noise and vibration, which the SITL model does not have.
- **Roll and pitch attitude ended unequal** (`MC_ROLL_P` 11.0, `MC_PITCH_P` 6.5). Pitch improvements stayed inside its larger repeat noise, so the rule (accept only a change larger than twice the noise) kept Default. A follow-up could tie the two axes together.
- The gains are saved as the profile **`Simulated-PID-Gains`** (`px4_tuning_parameters`), so they can be selected in Main.mlx or flown by name.

Figure: `RMSETuning_GainEvolution_*`: every gain flown in each tuning flight, relative to Default. × marks the robustness probes; the large outlined markers are the selected gains.

## 2. Final comparison on the figure-8

**Test:** the same 4 m figure-8 at 5 m altitude as on 12 November, with the heading along the path and velocity feedforward. Every gain set flew 3 times with `run_mission`; all 15 flights were valid.

| Mean of 3 flights | PX4 Default | Hand-tuned | **RMSE-tuned (new)** | RMSE (earlier) | LQR + integral |
|---|---:|---:|---:|---:|---:|
| Tracking error, RMS 3D [cm] | 34.8 | 19.5 | **11.5** | 12.0 | 19.2 |
| Distance from the path, RMS [cm] | 23.6 | 7.1 | **4.9** | 5.5 | 14.5 |
| Lag behind the reference [s] | 0.28 | 0.22 | 0.12 | 0.12 | **0.04** |
| Altitude error, RMS [cm] | 4.3 | 8.4 | 4.6 | 4.6 | **4.3** |
| Heading error, RMS [deg] | 1.6 | **1.2** | 1.4 | **1.1** | 8.0 |
| Max tilt [deg] | 7.4 | 5.7 | **5.5** | 5.7 | 6.2 |
| Motor-command rate, RMS [1/s] | 11.0 | 9.4 | 1.3 | 1.9 | **0.5** |
| Composite score (lower is better) | 98.6 | 51.4 | **30.9** | 32.4 | 65.1 |

The repeat spread is below 1 % for the PID sets and about 2 % for the LQR (`FinalComparison.csv` has the standard deviations).

**Findings:**
- **The new RMSE-tuned gains fly best.** Against the hand-tuned gains: tracking error −41 %, path deviation −31 %, composite score −40 %, and 7× smoother motors.
- **The automatic method reproduces and slightly beats the earlier RMSE campaign** (score 30.9 vs 32.4), starting from Default only and without any hand-tuned reference.
- **Hand-tuned:** good horizontal tracking, but the worst altitude error (8.4 cm). Its vertical velocity gains are half of Default.
- **LQR + integral:** the best lag, altitude and motor smoothness, but 3× the path deviation of the tuned PID. Its weak point is still the slow yaw axis: heading error 8° RMS, up to 17° in the turns.

Figures:
- `FinalComparison_Bars_*`: bar charts with the individual flights as dots.
- `FinalComparison_Paths_*`: top view, tracking error and heading error over the flight (first flight of each set).

## 3. Code and documentation

**New layout:**
- `parameters/`: vehicle and simulation parameters;
- `px4/`: PX4 connection helpers;
- `controller/`: LQR/LQI;
- `flight/`: missions and scoring;
- `flight/auto_tuning/`.

`project_paths` sets up every folder.

**Missions from code:** `run_mission(plan, Gains=..., Controller=..., Repeats=...)` flies and scores any mission, and `compare_missions` makes the figures and table above. This replaces the older one-off validation script.

**Custom controller design:** `design_custom_controller` reproduces the flown LQI gains and can write the parameter file.

**Help and README:**
- every function has MATLAB `help` text;
- the README explains how to use and how each feature works (missions, tuning and its score, custom controller).
