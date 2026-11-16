# Results, 12 November 2026

This page summarises the work since the last meeting. Every figure is given twice:
- `_CB`: colour-blind safe (viridis colours plus line styles);
- `_RG`: standard red/green/blue, not colour-blind safe.

Each figure is a PDF (vector, for LaTeX) and a PNG (300 dpi).

All flights are PX4 v1.14.3 SITL against the Simscape plant. They are scored on the plant's physical truth, not on PX4's own estimate.

These results were obtained with an earlier vehicle configuration, not with the unit drone this repository now models. They show what the tools do; the current parameters do not reproduce these numbers.

## 1. PID gains on a figure-8: which tuning flies best?

**Test:**
- A 4 m figure-8 (lemniscate of Gerono) at 5 m altitude, flown in 20 s.
- The heading follows the direction of travel.
- Every controller gets the same reference: position, analytic velocity feedforward, heading and yaw rate.
- Three repeat flights per gain set.

| Mean of 3 flights | PX4 Default | Hand-tuned | Guide-tuned | **RMSE-tuned** |
|---|---:|---:|---:|---:|
| Tracking error, RMS 3D [cm] | 36.0 | 20.5 | 35.1 | **12.6** |
| Distance from the path, RMS [cm] | 24.5 | 8.5 | 23.8 | **5.9** |
| Lag behind the reference [s] | 0.26 | 0.22 | 0.26 | **0.12** |
| Altitude error, RMS [cm] | **4.4** | 8.5 | 4.5 | **4.4** |
| Max tilt [deg] | 7.6 | 5.9 | 7.4 | **5.7** |
| Motor-command rate, RMS [1/s] | 11.1 | 9.5 | **0.9** | 1.9 |
| Composite score (lower is better) | 107.4 | 63.0 | 104.9 | **41.0** |

The repeat spread is below 1 % on every metric; the dots in the bar charts are the individual flights.

**The gain sets:**
- **Hand-tuned:** the gains from the real drone.
- **Guide-tuned:** an automatic run following the PX4 tuning guide (section 3).
- **RMSE-tuned:** an earlier automatic campaign that minimised an RMSE step-response score.

**Findings:**
- **RMSE-tuned flies best on every tracking metric.** It also keeps the motors 5× smoother than Default and hand-tuned.
- **Guide-tuned differs little from Default.** The guide's rule keeps the horizontal velocity and position gains at Default, because they already overshoot by more than 1 %.
- **The horizontal loops decide the result.** `MPC_XY_VEL_P_ACC` is 4.5 for RMSE, 4 for hand-tuned, and 1.8 for Default and guide-tuned.
- **Conclusion: the RMSE score selects better gains.** The new tuning method is built on it (section 6).

Figures:
- `PathComparison_PID_*`: top view, tracking error and heading error over the flight.
- `PID_Repeats_*`: bar charts with the individual flights as dots.

Note: these flights were flown before the magnetometer fix (section 4). The roughly 7° heading-RMS floor is that sensor error and is the same for all gain sets.

## 2. Own controller inside PX4: LQR, then LQR with integral action

**How it runs in PX4:**
- A 12-state discrete LQR at 100 Hz runs inside PX4 (`PX4_Receive`, offboard actuator mode).
- PX4's own PID loops are bypassed. The LQR writes the motor commands directly.
- It is switched on with `CUSTOM_CONTROLLER = 1` in `Main.mlx`.

**Design:**
- The linear model is the linearised Simscape plant: position, velocity, attitude and body rates.
- Weights: Bryson, R = 1000.
- Linear check on the full plant with motor dynamics: stable, with a phase margin at the motors of 39° / 38° / 27° for 0 / 1 / 2 samples of extra delay.

**Integral action:**
- Four integrators were added, on the N, E, D and yaw errors (LQI).
- Weights: 0.5 m·s and 0.5 rad·s.
- Phase margin at the motors: 34° / 33° / 25°.
- In the linear model, a constant disturbance is removed within about 5 s instead of leaving a permanent offset: a hover-thrust bias of 0.01 leaves 32 cm, a steady wind push 12 cm, a yaw-torque bias 8°.
- In flight, the integrator only runs after the takeoff climb, freezes while a motor saturates, and is capped.

Same figure-8, after the magnetometer fix:

| One flight each | LQR | **LQR + integral** | PID, RMSE gains |
|---|---:|---:|---:|
| Tracking error, RMS 3D [cm] | 28.2 | **18.9** | 12.1 |
| Distance from the path, RMS [cm] | 16.7 | **14.5** | 5.4 |
| Lag [s] | 0.26 | **0.02** | 0.12 |
| Altitude error, RMS [cm] | 8.0 | **4.5** | 4.5 |
| Heading error, RMS [deg] | 7.8 | 8.3 | 1.2 |
| Motor-command rate, RMS [1/s] | 0.35 | 0.44 | 1.9 |
| Composite score | 86.5 | **64.4** | 32.7 |

**Comparison:**
- The integral action roughly halves the altitude error and removes the 8 cm hover offset of the plain LQR.
- The tuned PID still tracks the path better.
- The LQR's heading response is the weak point. It comes from the slow yaw axis of the design, which was deliberately not changed in this step.

Figures:
- `PathComparison_LQI_*`: paths.
- `LQI_PoleMap_*`: closed-loop poles of both designs on the full plant with motor dynamics. The integrators add slow real poles near z = 1. The slowest time constant goes from 1.2 s (LQR) to 2.9 s (LQI), which is the integral action removing a steady offset.

## 3. Automatic tuning following the PX4 tuning guide

**The run:**
- Started from PX4 Default on the fixed plant.
- 31 flights, all valid, in 10.7 h wall time.
- Each flight excites all axes of one loop group.

**Results:**
- **Default rate gains oscillate on this airframe:** 43 % roll-rate and 60 % pitch-rate overshoot. The guide's fix (P and D × 0.81, then K × 0.9) brings this down to 8–9 %.
- **Vertical velocity:** P went to 3.6× Default.
- **Horizontal loops:** left at Default.

Figures:
- `GainEvolution_*`: every gain flown in each tuning flight, relative to Default. The large outlined markers are the selected gains.
- `TimeDistribution` and `TimeByGroupAndPhase`: where the wall time went. Step tests take 73 %, PX4 build and start 12 %, takeoff 12 %.

## 4. Heading estimate fixed

**The problem:** PX4's heading estimate was up to 10° off the true heading: 9.8° in hover, varying during turns.

**Cause:**
- The plant's magnetometer used MATLAB's default magnetic field: 5° west, pointing upwards, 32 µT.
- That field was unrelated to the GPS position, which is in Denmark, where the field is about 5° east, 70° downwards and 51 µT.
- PX4 looks up the declination for the reported GPS position, so the two disagreed by about 10°.

**Fix:**
- The home position is now set once (`SimParams.HomeLLA`). The GNSS block uses it.
- The magnetometer field is the World Magnetic Model at that position and date, so it is consistent anywhere.

**Validation:** same gains, same figure-8:

| | Before | After |
|---|---|---|
| Heading error | mean 6.3°, up to 10° | −0.41 ± 0.24° |
| Heading RMS (score) | 7.2° | 1.15° |
| Path deviation | 5.9 cm | 5.4 cm |

Figure: `HeadingFix_*`.

## 5. Faster simulation

**Timing:** a figure-8 flight was timed under several settings:
- **PX4_Receive scopes off:** the flight took 141 s instead of 297 s, a 52 % cut, with identical results. In external mode, each of the 9 scopes streams its data from PX4 on every step. The scopes only display data; all logged and scored signals are kept.
  - Automated runs (tuning, comparisons) now switch the scopes off in memory and back on at the end.
  - Interactive runs from `Main.mlx` keep them.
- **Plant signal logging decimated ×4:** no measurable gain in a full flight.
- **Tunable parameters to avoid the PX4 rebuild:** no gain. PX4 still rebuilds when gain values change, so about 160 s per flight remains.

The next tuning run fits roughly twice as many flights into the same time.

## 6. Next: tuning with the RMSE score (running until Friday)

The tuner now has a second method, `run_px4_guide_tuning("all",Method="rmse")`:
- **Score:** each axis's gains are searched to minimise J = RMSE/A + 0.30·max|e|/A + 0.25·overshoot on the step tests, the score that found the best gains in section 1.
- **Acceptance:** a change is accepted only if it beats the best score by more than twice the repeat noise, measured from 3 baseline flights.
- **Robustness:** after each loop group, every loop gain is flown ×1.3. An axis that oscillates is backed off ×0.85, which gives a gain margin of at least 1.3.
- **Time budget:** at most 22 h.

The resulting gains will be compared with hand-tuned and Default on repeated figure-8 flights.
