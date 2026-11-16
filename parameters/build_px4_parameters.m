function PX4Params = build_px4_parameters(Config, Vehicle, SimParams)
%BUILD_PX4_PARAMETERS
% Build PX4 parameters consistent with the selected actuator model.
%
% Plant motor order:
%
%   M1 = front-right, CCW
%   M2 = front-left,  CW
%   M3 = rear-right,  CW
%   M4 = rear-left,   CCW

PX4Params = struct();

%% ------------------------------------------------------------------------
% Identification
% -------------------------------------------------------------------------

PX4Params.PropulsionID = ...
    Config.Propulsion.ID;


%% ------------------------------------------------------------------------
% Airframe
% -------------------------------------------------------------------------

PX4Params.CA_AIRFRAME = 0;

PX4Params.CA_ROTOR_COUNT = ...
    size(Vehicle.RotorPositions,1);


%% ------------------------------------------------------------------------
% Rotor ordering
%
% Vehicle order:
%
%   1 = front-right
%   2 = rear-right
%   3 = rear-left
%   4 = front-left
%
% Plant motor order:
%
%   M1 = front-right
%   M2 = front-left
%   M3 = rear-right
%   M4 = rear-left
% -------------------------------------------------------------------------

PX4RotorOrder = ...
    [1 4 2 3];

rotorPositionsPX4 = ...
    Vehicle.RotorPositions(PX4RotorOrder,:);


%% ------------------------------------------------------------------------
% Rotor geometry
% -------------------------------------------------------------------------

PX4Params.CA_ROTOR_PX = ...
    rotorPositionsPX4(:,1);

PX4Params.CA_ROTOR_PY = ...
    rotorPositionsPX4(:,2);

PX4Params.CA_ROTOR_PZ = ...
    rotorPositionsPX4(:,3);


%% ------------------------------------------------------------------------
% Rotor thrust coefficient
%
% PX4:
%
%   T = CT*u^2
%
% Our model:
%
%   omega = u*CommandMaxOmega
%   T     = kT*omega^2
%
% Therefore CT is simply the thrust at u = 1.
% -------------------------------------------------------------------------

if ~isfinite(SimParams.CommandMaxThrustPerMotor)

    error( ...
        "PX4 actuator scale is undefined for '%s'.", ...
        Config.Propulsion.Name);

end

PX4Params.CA_ROTOR_CT = ...
    SimParams.CommandMaxThrustPerMotor * ...
    ones(PX4Params.CA_ROTOR_COUNT,1);


%% ------------------------------------------------------------------------
% Rotor moment coefficient
% -------------------------------------------------------------------------

PX4Params.KM_Magnitude = ...
    SimParams.KM_Magnitude;


%% Rotor rotation directions

PX4Params.RotorSpinDirection = [ ...
     1;     % M1 front-right CCW
    -1;     % M2 front-left  CW
    -1;     % M3 rear-right  CW
     1];    % M4 rear-left   CCW

PX4Params.CA_ROTOR_KM = ...
    PX4Params.RotorSpinDirection .* ...
    PX4Params.KM_Magnitude;


%% ------------------------------------------------------------------------
% Hover / thrust model
% -------------------------------------------------------------------------

PX4Params.MPC_THR_HOVER = ...
    SimParams.HoverThrust / ...
    sum(PX4Params.CA_ROTOR_CT);

PX4Params.MPC_USE_HTE = 0;

% Pure quadratic command-to-thrust model
PX4Params.THR_MDL_FAC = 1;


%% ------------------------------------------------------------------------
% Velocity limits
% -------------------------------------------------------------------------

PX4Params.MPC_XY_VEL_MAX = ...
    10;

PX4Params.MPC_Z_VEL_MAX_UP = ...
    20;

PX4Params.MPC_Z_VEL_MAX_DN = ...
    5;


%% ------------------------------------------------------------------------
% Angular limits
% -------------------------------------------------------------------------

PX4Params.MPC_TILTMAX_AIR = ...
    35;


%% ------------------------------------------------------------------------
% Arming
% -------------------------------------------------------------------------

PX4Params.COM_DISARM_PRFLT = ...
    -1;


%% ------------------------------------------------------------------------
% Summary
% -------------------------------------------------------------------------

fprintf("\n");
fprintf("========================================\n");
fprintf(" PX4 Parameters\n");
fprintf("========================================\n");

fprintf(" Rotor count       : %d\n", ...
    PX4Params.CA_ROTOR_COUNT);

fprintf(" Rotor CT          : %.4f N\n", ...
    PX4Params.CA_ROTOR_CT(1));

fprintf(" |KM|              : %.6f m\n", ...
    PX4Params.KM_Magnitude);

fprintf(" MPC_THR_HOVER     : %.4f\n", ...
    PX4Params.MPC_THR_HOVER);


%% Rotor table

Rotor = ...
    (0:PX4Params.CA_ROTOR_COUNT-1)';

Motor = [ ...
    "M1";
    "M2";
    "M3";
    "M4"];

RotorTable = table( ...
    Rotor, ...
    Motor, ...
    PX4Params.CA_ROTOR_PX(:), ...
    PX4Params.CA_ROTOR_PY(:), ...
    PX4Params.CA_ROTOR_KM(:), ...
    PX4Params.CA_ROTOR_CT(:), ...
    'VariableNames', ...
    {'Rotor','Motor','PX','PY','KM','CT'});

disp(RotorTable);

fprintf(" MPC_XY_VEL_MAX    : %.2f\n", ...
    PX4Params.MPC_XY_VEL_MAX);

fprintf(" MPC_Z_VEL_MAX_UP  : %.2f\n", ...
    PX4Params.MPC_Z_VEL_MAX_UP);

fprintf(" MPC_Z_VEL_MAX_DN  : %.2f\n", ...
    PX4Params.MPC_Z_VEL_MAX_DN);

fprintf(" MPC_TILTMAX_AIR   : %.2f\n", ...
    PX4Params.MPC_TILTMAX_AIR);

fprintf("========================================\n\n");

end