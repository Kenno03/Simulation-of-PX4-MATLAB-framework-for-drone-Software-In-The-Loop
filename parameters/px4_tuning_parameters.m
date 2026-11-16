function Tune = px4_tuning_parameters(PX4TuneChoice)
%PX4_TUNING_PARAMETERS Selectable flight-controller profiles (PX4 gains, gyro filters, controller).
%   Tune = px4_tuning_parameters(name) returns the profile NAME as a struct of PX4 parameters
%   (PID gains of the rate, attitude, velocity and position loops and the gyro filters) plus
%   Tune.CustomController: 0 = PX4's PID loops fly the drone, 1 = the custom LQI controller in
%   PX4_Receive flies it (Main.mlx sets CUSTOM_CONTROLLER from this field).
%   Tune = px4_tuning_parameters() returns the list of profile names (Main.mlx drop-down).
%   Tune.ValidatedFor is the setup a profile was tuned on (mass, inertia, maximum thrust); empty for
%   the generic PX4 defaults. check_gain_validity warns when the current setup differs.
%
%   These are PX4's parameters, not the plant's internal motor-speed PI controller.
%
%   Profiles:
%     "Default"              PX4 SITL default gains.
%     "HandTuned_RealDrone"  gains from physical flight tuning of a real drone.
%     "Simulated-PID-Gains"  result of the automatic RMSE tuning from Default in this SITL setup
%                            (run_px4_guide_tuning("all",Method="rmse"), 11-12 Nov 2026, 69 flights).
%                            Tuned for the simulation, not checked on a real drone.
%   The hand-tuned and simulated gains were tuned on a different vehicle than the unit drone of this
%   repository, so check_gain_validity warns when they are selected.
%     "LQI"                  the custom LQR with integral action (custom_controller_parameters)
%                            flies the drone; PX4's PID loops are bypassed and keep Default gains.

TuneOptions = [ ...
    "Default"
    "HandTuned_RealDrone"
    "Simulated-PID-Gains"
    "LQI"
];

if nargin == 0
    Tune = TuneOptions;
    return
end

PX4TuneChoice = string(PX4TuneChoice);

switch PX4TuneChoice

    case "Default"

        Tune = defaultProfile();

    case "HandTuned_RealDrone"

        Tune = struct();
        Tune.ID = "HandTuned_RealDrone";
        Tune.Name = "Hand Tuned Real Drone";
        Tune.Source = "Physical flight tuning";

        % attitude
        Tune.MC_PITCH_P = 8.0;
        Tune.MC_ROLL_P = 8.0;

        % pitch rate
        Tune.MC_PITCHRATE_P = 0.11;
        Tune.MC_PITCHRATE_D = 0.0023;

        % yaw rate
        Tune.MC_YAWRATE_P = 0.38;
        Tune.MC_YAWRATE_I = 0.15;
        Tune.MC_YAWRATE_K = 0.70;

        % horizontal position and velocity
        Tune.MPC_XY_P = 0.80;
        Tune.MPC_XY_VEL_P_ACC = 4.00;
        Tune.MPC_XY_VEL_I_ACC = 0.00;
        Tune.MPC_XY_VEL_D_ACC = 0.15;

        % vertical position and velocity
        Tune.MPC_Z_P = 0.90;
        Tune.MPC_Z_VEL_P_ACC = 2.00;
        Tune.MPC_Z_VEL_I_ACC = 0.20;
        Tune.MPC_Z_VEL_D_ACC = 0.02;

        % gyro filters
        Tune.IMU_DGYRO_CUTOFF = 25;
        Tune.IMU_GYRO_CUTOFF = 50;
        Tune.IMU_GYRO_DNF_BW = 20;
        Tune.IMU_GYRO_DNF_EN = 1;
        Tune.IMU_GYRO_DNF_HMC = 2;
        Tune.IMU_GYRO_DNF_MIN = 75;

        Tune.ValidatedFor = tunedSetup();

    case "Simulated-PID-Gains"

        % Default with the gains the RMSE tuning changed (session
        % PX4_20261111_161706_rmse_tune). The rate gains were stepped x0.77 and
        % then backed off x0.85 after the x1.3 robustness probe; three gains
        % ended at PX4's upper parameter limit (MC_YAW_P, MPC_XY_VEL_P_ACC,
        % MPC_Z_VEL_P_ACC). The pitch-attitude search found no significant
        % improvement, so MC_PITCH_P stayed at Default.
        Tune = defaultProfile();
        Tune.ID = "Simulated-PID-Gains";
        Tune.Name = "Simulated PID gains (RMSE-tuned in SITL)";
        Tune.Source = "Automatic RMSE tuning from Default, 69 SITL flights";

        % Rounded to PX4's resolution (flown: K 0.85/1.3 = 0.653846, MC_ROLL_P 10.985, MPC_XY_P 1.6055).
        Tune.MC_ROLL_P = 10.99;
        Tune.MC_YAW_P = 5.0;
        Tune.MC_ROLLRATE_K = 0.6538;
        Tune.MC_PITCHRATE_K = 0.6538;
        Tune.MC_YAWRATE_K = 0.6538;
        Tune.MPC_XY_P = 1.61;
        Tune.MPC_XY_VEL_P_ACC = 5.0;
        Tune.MPC_Z_P = 1.5;
        Tune.MPC_Z_VEL_P_ACC = 15.0;
        Tune.ValidatedFor = tunedSetup();

    case "LQI"

        % The custom controller flies; PX4 still receives (unused) Default PID gains.
        Tune = defaultProfile();
        Tune.ID = "LQI";
        Tune.Name = "Custom controller: LQR with integral action";
        Tune.Source = "custom_controller_parameters (PX4 PID loops bypassed, Default gains)";
        Tune.CustomController = 1;

    otherwise

        error("Unknown PX4 tuning profile: %s. Options: %s", ...
            PX4TuneChoice, strjoin(TuneOptions, ", "));

end

% Complete every profile without changing any explicitly supplied value.
% Verified against installed PX4 mc_rate_control_params.c / mc_att_control_params.c.
fallback = struct('MC_ROLLRATE_P',0.15,'MC_ROLLRATE_I',0.20, ...
    'MC_ROLLRATE_D',0.003,'MC_ROLLRATE_K',1.0,'MC_PITCHRATE_I',0.20, ...
    'MC_PITCHRATE_K',1.0,'MC_YAWRATE_D',0.0,'MC_YAW_P',2.8,'CustomController',0);
for field = string(fieldnames(fallback))'
    if ~isfield(Tune,field), Tune.(field) = fallback.(field); end
end
assert(all(isfield(Tune,px4_controller_gain_names())),'Incomplete controller gain profile.');
% Setup the gains were validated on (check_gain_validity); empty = generic gains (PX4 defaults).
if ~isfield(Tune,'ValidatedFor'), Tune.ValidatedFor = []; end

% Summary
fprintf("\n========================================\n");
fprintf(" PX4 Tuning Profile\n");
fprintf("========================================\n");
fprintf(" Profile             : %s\n", Tune.Name);
fprintf(" Source              : %s\n", Tune.Source);
if Tune.CustomController
    fprintf(" Flight controller   : custom LQI (CUSTOM_CONTROLLER = 1)\n");
else
    fprintf(" Flight controller   : PX4 PID loops\n");
end
fprintf("\n Controller gains (feedforward excluded):\n");
for field = px4_controller_gain_names()
    fprintf("   %-22s : %.6g\n", field, Tune.(field));
end
fprintf("\n Gyro filters:\n");
fprintf("   DGYRO cutoff      : %.1f Hz\n", Tune.IMU_DGYRO_CUTOFF);
fprintf("   GYRO cutoff       : %.1f Hz\n", Tune.IMU_GYRO_CUTOFF);
fprintf("   Dynamic notch     : %d\n", Tune.IMU_GYRO_DNF_EN);
fprintf("========================================\n\n");

end

function V = tunedSetup()
% Setup the hand-tuned and Simulated-PID-Gains profiles were tuned on. That vehicle is not part of
% this repository, so its mass, inertia and thrust are unknown here (NaN): check_gain_validity then
% reports that the gains were tuned for a different vehicle.
V = struct('Mass',NaN,'Inertia',[NaN NaN NaN],'MaxThrust',NaN);
end

function Tune = defaultProfile()
% PX4 SITL default values (the base of the "Default", "Simulated-PID-Gains" and "LQI" profiles).
Tune = struct();
Tune.ID = "Default";
Tune.Name = "PX4 SITL Default";
Tune.Source = "Recorded PX4 SITL defaults (static profile)";

% attitude
Tune.MC_PITCH_P = 6.5;
Tune.MC_ROLL_P = 6.5;

% pitch rate
Tune.MC_PITCHRATE_P = 0.15;
Tune.MC_PITCHRATE_D = 0.003;

% yaw rate
Tune.MC_YAWRATE_P = 0.20;
Tune.MC_YAWRATE_I = 0.10;
Tune.MC_YAWRATE_K = 1.00;

% horizontal position and velocity
Tune.MPC_XY_P = 0.95;
Tune.MPC_XY_VEL_P_ACC = 1.80;
Tune.MPC_XY_VEL_I_ACC = 0.40;
Tune.MPC_XY_VEL_D_ACC = 0.20;

% vertical position and velocity
Tune.MPC_Z_P = 1.00;
Tune.MPC_Z_VEL_P_ACC = 4.00;
Tune.MPC_Z_VEL_I_ACC = 2.00;
Tune.MPC_Z_VEL_D_ACC = 0.00;

% gyro filters
Tune.IMU_DGYRO_CUTOFF = 30;
Tune.IMU_GYRO_CUTOFF = 40;
Tune.IMU_GYRO_DNF_BW = 15;
Tune.IMU_GYRO_DNF_EN = 0;
Tune.IMU_GYRO_DNF_HMC = 3;
Tune.IMU_GYRO_DNF_MIN = 25;
end
