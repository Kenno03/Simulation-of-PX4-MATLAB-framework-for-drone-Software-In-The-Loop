function [lower,upper,resolution]=px4_controller_gain_limits(name)
%PX4_CONTROLLER_GAIN_LIMITS Allowed range and resolution of a PX4 gain (PX4 v1.14.3 parameter definitions).
%   [lower,upper] = px4_controller_gain_limits(name) returns the bounds the tuners clip to, for example
%   px4_controller_gain_limits("MC_PITCH_P") -> 0, 12. A missing published maximum is Inf.
%   [lower,upper,resolution] = px4_controller_gain_limits(name) also returns the gain's resolution:
%   10^-decimals from the parameter's @decimal metadata, the number of decimals PX4/QGroundControl
%   shows and accepts (for example 0.01 for MC_PITCH_P, 0.0001 for MC_ROLLRATE_D). The tuners round
%   every gain they fly to this grid, so the result can be entered in PX4 exactly.
switch string(name)
    %                                   lower   upper   decimals
    case "MC_ROLLRATE_K",    v={0.01,  5,      4};
    case "MC_ROLLRATE_P",    v={0.01,  0.5,    3};
    case "MC_ROLLRATE_I",    v={0,     Inf,    3};
    case "MC_ROLLRATE_D",    v={0,     0.01,   4};
    case "MC_PITCHRATE_K",   v={0.01,  5,      4};
    case "MC_PITCHRATE_P",   v={0.01,  0.6,    3};
    case "MC_PITCHRATE_I",   v={0,     Inf,    3};
    case "MC_PITCHRATE_D",   v={0,     Inf,    4};
    case "MC_YAWRATE_K",     v={0,     5,      4};
    case "MC_YAWRATE_P",     v={0,     0.6,    2};
    case "MC_YAWRATE_I",     v={0,     Inf,    2};
    case "MC_YAWRATE_D",     v={0,     Inf,    2};
    case "MC_ROLL_P",        v={0,     12,     2};
    case "MC_PITCH_P",       v={0,     12,     2};
    case "MC_YAW_P",         v={0,     5,      2};
    case "MPC_XY_P",         v={0,     2,      2};
    case "MPC_XY_VEL_P_ACC", v={1.2,   5,      2};
    case "MPC_XY_VEL_I_ACC", v={0,     60,     3};
    case "MPC_XY_VEL_D_ACC", v={0.1,   2,      3};
    case "MPC_Z_P",          v={0,     1.5,    2};
    case "MPC_Z_VEL_P_ACC",  v={2,     15,     2};
    case "MPC_Z_VEL_I_ACC",  v={0.2,   3,      3};
    case "MPC_Z_VEL_D_ACC",  v={0,     2,      3};
    otherwise,error('Unknown controller gain: %s.',name);
end
[lower,upper,decimals]=v{:};resolution=10^-decimals;
end
