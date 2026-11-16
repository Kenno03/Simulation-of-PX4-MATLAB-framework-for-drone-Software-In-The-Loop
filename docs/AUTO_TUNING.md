# Automatic PID tuning

The auto-tuner tunes all 23 PID gains of PX4's cascaded controller (rates, attitude, velocity, position) in PX4 SITL against the Simscape plant. It needs no hand-tuned values: it starts from PX4's Default gains and flies test flights until the gains stop improving or the time budget runs out.

There are two search methods with the same flights and safety checks:
- **RMSE** (recommended): minimises an RMSE-based step-response score.
- **Guide**: follows the rules of the PX4 tuning guide.

## How to use

1. Run Main.mlx's setup sections with the **Default** profile.
2. Start the tuner, from the Command Window or with menu choice **18** (RMSE) or **17** (guide) in Main.mlx's tuning section:

```matlab
TuningSession = run_px4_guide_tuning("all", Method="rmse", MaxHours=22);
```

Options:

| Option | Default | Meaning |
|---|---|---|
| first argument | `"all"` | loop groups to tune: `"all"` or any of `"rates"`, `"attitude"`, `"velocity"`, `"position"` |
| `Method` | `"guide"` | `"rmse"` (recommended) or `"guide"` |
| `MaxHours` | 24 | wall-time budget; the tuner stops starting new flights when it is used up |
| `Repeats` | 3 | baseline flights per group (measure the repeat noise) |
| `Mode` | `"tune"` | `"validate"` flies the current gains once per group without changing them |

3. Use the result:

```matlab
PX4Tune = TuningSession.BestGains;                 % fly the tuned gains now
A = run_mission(plan, Gains=TuningSession.BestGains, Repeats=3);   % or compare them on a path
```

A result worth keeping becomes a named profile in `parameters/px4_tuning_parameters.m`; the result of the November 2026 run is the profile **`Simulated-PID-Gains`**.

4. To reload a finished session:

```matlab
L = load(fullfile(folder, "session.mat")); TuningSession = L.Session;
```

**Run one experiment at a time**, and never start another simulation while the tuner runs. The plant holds the TCP port 4560 that PX4 connects to.

## How it works

### Flights

The loop groups are tuned from the inside out:

**rates → attitude → velocity → position**

Each group is tuned with the inner loops already tuned.

Every flight follows the same sequence:
1. Takes off to 10 m and waits until the drone has settled (altitude reached, climb rate below 1 m/s).
2. From 40 s, gives each axis of the group ten alternating step commands, one axis after the other. Rates and attitude test roll, pitch and yaw; velocity and position test horizontal and vertical. One takeoff therefore serves every axis of the group.
3. Stops and scores each axis from the plant's true state. PX4's own estimate is not used.

Each group then runs these phases:
1. **Baseline:** 3 flights with unchanged gains. Their mean is the start score and their spread is the repeat noise.
2. **Search:** one flight per proposal, with all axes of the group searching at the same time (at most 20 flights per group).
3. **Robustness probe** (RMSE method): one flight with every loop gain of the group ×1.3.

After the last group, a **final check** flies every group once with the selected gains.

### The score (RMSE method)

Each axis is scored on its step responses:

**J = RMSE / A + 0.30 · max|e| / A + 0.25 · overshoot**

- A is the step amplitude.
- RMSE is the root-mean-square tracking error over the steps.
- max|e| is the largest error.
- The overshoot term is relative.

Lower is better. Dividing by A makes axes with different units comparable.

### The search (RMSE method, `px4_rmse_axis`)

Each axis searches only its own gains, the "knobs":

| Loop | Knobs |
|---|---|
| Roll / pitch rate | K (overall loop gain), D, I |
| Yaw rate | K, I |
| Attitude | P |
| Velocity | P, I, D |
| Position | P |

Rate P is scaled through K, so the P : I : D ratio of a rate loop changes only through D and I.

- **Step:** a knob is multiplied by 1.3. If that doesn't help, it is divided by 1.3 instead. While a direction keeps improving the score, the search continues in it.
- **Acceptance:** a change is kept only if the score improves by more than the repeat noise, which is twice the standard deviation of the baseline flights. This stops the tuner from chasing noise.
- **Oscillation gate:** a response that oscillates (oscillation metric above 0.15), goes unstable or fails a validity check is always rejected, whatever its score.
- **Refinement:** after a full cycle over all knobs without an accepted change, the step is refined to ×1.14. After another cycle without improvement, or after 14 proposals, the axis is done.
- **PX4 resolution:** every gain is rounded to the number of decimals PX4 accepts for that parameter, so the result can be entered in PX4 exactly. The decimals come from PX4 v1.14.3's parameter definitions, in `px4_controller_gain_limits`. Examples: 2 decimals for `MC_ROLL_P`, 4 for `MC_ROLLRATE_D`. Gains are also clipped to PX4's allowed range.
- **No repeated flights:** a step that rounds back to the current value, or to gains the axis has already flown, is skipped without a flight.

### Robustness probe

After the search of a group, every loop gain of the group is flown ×1.3:
- **Probe fails:** an axis that then oscillates or goes unstable is backed off ×0.85, and the probe is repeated, at most 3 times.
- **Probe passes:** the gains have a gain margin of at least 1.3 against oscillation.

### Guide method (`px4_guide_axis`)

The guide method follows the PX4 tuning guide step by step:
1. Remove oscillation.
2. Balance P and D.
3. Raise the gains until overshoot or oscillation starts, then back off.

It uses the same flights, checks and outputs as the RMSE method. On this airframe it keeps the outer loops at Default, because Default already overshoots there by more than 1 %. The RMSE method tunes them further.

### Safety and validity

A flight is only scored if it is complete and safe:
- all steps were flown;
- the drone stayed above the minimum altitude;
- body rates stayed limited;
- the drone recovered after each step;
- no motor saturated for long (`score_px4_tuning_candidate`).

An execution failure, such as the rare PX4 start-up stall at about 16 s, is retried once and never scored as a controller result.

## Output

One folder per session: `TuningSessions/PX4_<time>_<method>_<mode>/`. The folder is git-ignored.

| File | Content |
|---|---|
| `session.mat` | everything: settings, start and best gains, flight table, decisions |
| `guide_flights.csv` | one row per axis and flight: gains, score, validity, wall time |
| `guide_final_check.csv` | the final check with the selected gains |
| `guide_decisions.txt` | every accepted or rejected step, probe and back-off, in plain text |
| `GainEvolution_CB/RG.pdf/.png` | every gain over the flight numbers, relative to Default (`plot_gain_evolution`); × marks the robustness probes |
| `Flight###_*` | raw data, step-response figures and trial reports of each flight |

The session is saved after every flight. If MATLAB stops, the flights so far are kept.

## Time

Each flight takes about 13 minutes of wall time. The November 2026 RMSE run from Default needed 69 flights in 15.1 h. Where that time went:

| Activity | Share |
|---|---:|
| Step tests | 67 % |
| PX4 build and start (rebuilt for every new gain set) | 20 % |
| Takeoff and settling | 10 % |
| Stopping, exporting and scoring | 3 % |

The `PX4_Receive` scopes are switched off during tuning, which halves the flight time with identical logged data. Skipping already-flown gain sets would have saved 11 of the 48 search flights of that run, about 2.4 h.

## Results

On a figure-8, the gains from the November 2026 run (`Simulated-PID-Gains`) tracked better than the hand-tuned and Default gains. Compared with hand-tuned:
- RMS tracking error 11.5 vs 19.5 cm;
- path deviation 4.9 vs 7.1 cm;
- 7 times smoother motor commands.

Details and figures: [docs/results_20261116](results_20261116/README.md).

## Limitations

- **Simulation only:** the SITL plant has no sensor noise or frame vibration, which on a real drone limit how high the gains can go.
- **Gains at PX4's limit:** in the November 2026 run, four gains ended at PX4's maximum (`MC_YAW_P`, `MPC_XY_VEL_P_ACC`, `MPC_Z_P`, `MPC_Z_VEL_P_ACC`). Check those first on hardware.
- **Independent axes:** roll and pitch are tuned separately. A symmetric airframe can therefore end with different roll and pitch gains when one axis's improvement stays inside the repeat noise.
- **Local search:** the search finds a good local result from Default, not a guaranteed global optimum.

## Other tuning tools

- **Single-loop sessions:** `run_px4_tuning`, menu choices 1–16. They tune or verify one loop per flight with an older joint finite-difference search; they are useful to check a single loop.
- **Offline tests:** `tests/test_guide_tuner.m` checks the tuner logic and runs complete dry runs with synthetic flights. No simulation is started.
