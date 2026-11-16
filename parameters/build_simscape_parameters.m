function SimscapeParams = build_simscape_parameters(Config, Vehicle, SimParams)
%BUILD_SIMSCAPE_PARAMETERS
% Parameters required specifically by the Simscape plant.

SimscapeParams = struct();


%% ------------------------------------------------------------------------
% Body geometry
% -------------------------------------------------------------------------

SimscapeParams.BodyRadius = ...
    Config.Drone.Body_Radius;

SimscapeParams.BodyHeight = ...
    Config.Drone.Body_Height;

SimscapeParams.BodyMass = ...
    Config.Drone.Body_Mass;


%% ------------------------------------------------------------------------
% Arm geometry
% -------------------------------------------------------------------------

SimscapeParams.ArmLength = ...
    Config.Drone.Arm_Length;

SimscapeParams.ArmRadius = ...
    Config.Drone.Arm_Radius;

SimscapeParams.ArmMass = ...
    Config.Drone.Arm_Mass;


%% ------------------------------------------------------------------------
% Arm positions
%
% Arm solid COM positions relative to the body origin.
%
% +x = forward
% +y = right
% +z = down
% -------------------------------------------------------------------------

ArmCenterDistance = ...
    SimscapeParams.BodyRadius + ...
    SimscapeParams.ArmLength/2;

a = ...
    ArmCenterDistance/sqrt(2);

% Existing Simscape arm vertical position
ArmZ = ...
    SimscapeParams.BodyHeight/2 - 3e-2;

SimscapeParams.ArmPosition1 = ...
    [ a,  a, ArmZ];      % Front-right

SimscapeParams.ArmPosition2 = ...
    [ a, -a, ArmZ];      % Front-left

SimscapeParams.ArmPosition3 = ...
    [-a,  a, ArmZ];      % Rear-right

SimscapeParams.ArmPosition4 = ...
    [-a, -a, ArmZ];      % Rear-left


%% ------------------------------------------------------------------------
% Arm orientation
%
% Simscape cylinder axis is local +z.
%
% Each arm is rotated 90 degrees to align the cylinder with the
% corresponding arm direction in the body xy-plane.
%
% Simscape arm numbering:
%
%   Arm 1 = front-right
%   Arm 2 = front-left
%   Arm 3 = rear-right
%   Arm 4 = rear-left
% -------------------------------------------------------------------------

SimscapeParams.ArmRotationAngle = ...
    90;                                     % [deg]

SimscapeParams.ArmRotationAxis1 = ...
    [-1  1  0] / sqrt(2);                  % Arm 1

SimscapeParams.ArmRotationAxis2 = ...
    [ 1  1  0] / sqrt(2);                  % Arm 2

SimscapeParams.ArmRotationAxis3 = ...
    [-1 -1  0] / sqrt(2);                  % Arm 3

SimscapeParams.ArmRotationAxis4 = ...
    [ 1 -1  0] / sqrt(2);                  % Arm 4


%% ------------------------------------------------------------------------
% Motor-holder orientation
%
% Existing model:
%
%   Angle = -P.Arm.RotationAngle
%   Axis  =  P.Arm.RotationAxisN
%
% The motor-holder transform therefore reverses the corresponding
% arm rotation.
% -------------------------------------------------------------------------

SimscapeParams.MotorHolderRotationAngle = ...
    -SimscapeParams.ArmRotationAngle;

SimscapeParams.MotorHolderRotationAxis1 = ...
    SimscapeParams.ArmRotationAxis1;

SimscapeParams.MotorHolderRotationAxis2 = ...
    SimscapeParams.ArmRotationAxis2;

SimscapeParams.MotorHolderRotationAxis3 = ...
    SimscapeParams.ArmRotationAxis3;

SimscapeParams.MotorHolderRotationAxis4 = ...
    SimscapeParams.ArmRotationAxis4;


%% ------------------------------------------------------------------------
% Motor physical geometry
% -------------------------------------------------------------------------

SimscapeParams.MotorMass = ...
    Config.Motor.Mass;

SimscapeParams.MotorRadius = ...
    Config.Motor.Radius;

SimscapeParams.MotorHeight = ...
    Config.Motor.Height;


%% ------------------------------------------------------------------------
% Propeller geometry
% -------------------------------------------------------------------------

SimscapeParams.PropellerDiameter = ...
    Config.Propeller.Diameter;

SimscapeParams.PropellerRadius = ...
    Config.Propeller.Radius;

SimscapeParams.PropellerMass = ...
    Config.Propeller.Mass;

% Selected CAD geometry files
SimscapeParams.PropellerGeometryCW = ...
    Config.Propeller.Geometry.CW;

SimscapeParams.PropellerGeometryCCW = ...
    Config.Propeller.Geometry.CCW;

% Propeller balancing
% The Simscape model attaches each STEP stand-in propeller at its hub
% frame (File Solid frame F1). Measured from that spin axis, the solid's
% centre of mass is off-axis by 1.37 mm (CCW file) and 0.13 mm (CW file),
% which produces a rotor-synchronous imbalance force and yaw-rate chatter
% at the rotor frequency. A real propeller is balanced, so the model
% inserts these translations between each rotor joint and its propeller
% solid, putting the centre of mass on the spin axis. Axial inertia is
% unchanged. Values: measured in-plane CoM offset of the STEP file, negated.
SimscapeParams.PropellerCADCentringCCW = ...
    [0.000198265, 0.00135739, 0];             % [m]

SimscapeParams.PropellerCADCentringCW = ...
    [-7.78234e-05, 0.000108314, 0];           % [m]


%% ------------------------------------------------------------------------
% Rotor locations
% -------------------------------------------------------------------------

SimscapeParams.RotorPositions = ...
    Vehicle.RotorPositions;


%% ------------------------------------------------------------------------
% Detailed aerodynamic geometry
%
% Geometry itself is reused from the physical component definitions.
% Only aerodynamic properties such as Cd and axis live under Aero.
% -------------------------------------------------------------------------

% Main body
SimscapeParams.BodyCd = ...
    Config.Drone.Aero.Body.Cd;

SimscapeParams.BodyAxis = ...
    Config.Drone.Aero.Body.Axis;


% Arms
SimscapeParams.ArmCd = ...
    Config.Drone.Aero.Arm.Cd;

SimscapeParams.ArmAxis = ...
    Config.Drone.Aero.Arm.Axis;


% Motor holders
SimscapeParams.MotorHolderCd = ...
    Config.Drone.Aero.MotorHolder.Cd;

SimscapeParams.MotorHolderAxis = ...
    Config.Drone.Aero.MotorHolder.Axis;


%% ------------------------------------------------------------------------
% Simscape Motor & Drive block
% -------------------------------------------------------------------------

SimscapeParams.MotorMaxTorque = ...
    Config.Motor.Simscape.MaxTorque;

SimscapeParams.MotorMaxPower = ...
    Config.Motor.Simscape.MaxPower;

SimscapeParams.MotorTorqueControlTimeConstant = ...
    Config.Motor.Simscape.TorqueControlTimeConstant;

SimscapeParams.MotorEfficiency = ...
    Config.Motor.Simscape.Efficiency;

SimscapeParams.MotorEfficiencyMeasurementSpeed = ...
    Config.Motor.Simscape.EfficiencyMeasurementSpeed;

SimscapeParams.MotorEfficiencyMeasurementTorque = ...
    Config.Motor.Simscape.EfficiencyMeasurementTorque;


%% ------------------------------------------------------------------------
% Validate detailed motor data
% -------------------------------------------------------------------------

motorValues = [ ...
    SimscapeParams.MotorMaxTorque
    SimscapeParams.MotorMaxPower
    SimscapeParams.MotorTorqueControlTimeConstant
    SimscapeParams.MotorEfficiency
    SimscapeParams.MotorEfficiencyMeasurementSpeed
    SimscapeParams.MotorEfficiencyMeasurementTorque
];

if any(~isfinite(motorValues))

    error( ...
        "No complete Simscape Motor & Drive parameters are defined for '%s'.", ...
        Config.Motor.Name);

end


%% ------------------------------------------------------------------------
% Motor speed controller
% -------------------------------------------------------------------------

SimscapeParams.MotorKp = ...
    Config.MotorController.Kp;

SimscapeParams.MotorKi = ...
    Config.MotorController.Ki;

SimscapeParams.MotorKd = ...
    Config.MotorController.Kd;


if ~isfinite(SimscapeParams.MotorKp) || ...
   ~isfinite(SimscapeParams.MotorKi)

    error( ...
        "No Simscape motor-speed controller tuning is defined for '%s'.", ...
        Config.Propulsion.Name);

end


%% ------------------------------------------------------------------------
% Normalized command scaling
% -------------------------------------------------------------------------

SimscapeParams.CommandMaxRPM = ...
    SimParams.CommandMaxRPM;

SimscapeParams.CommandMaxOmega = ...
    SimParams.CommandMaxOmega;


%% ------------------------------------------------------------------------
% Simscape Aerodynamic Propeller coefficients
%
% Simscape:
%
%   T = KT*rho*D^4*n^2
%
%   Q = KP*rho*D^5/(2*pi)*n^2
%
% with:
%
%   omega = 2*pi*n
%
% Our model:
%
%   T = kT*omega^2
%   Q = kQ*omega^2
% -------------------------------------------------------------------------

rho = ...
    SimParams.ReferenceDensity;

D = ...
    SimscapeParams.PropellerDiameter;


SimscapeParams.PropellerKT = ...
    4*pi^2 * SimParams.kT / ...
    (rho * D^4);


SimscapeParams.PropellerKP = ...
    8*pi^3 * SimParams.kQ / ...
    (rho * D^5);


%% ------------------------------------------------------------------------
% Body surface points
%
% Points on the cylindrical body surface used by the Simscape model.
%
% +x = forward
% +y = right
% +z = down
% -------------------------------------------------------------------------

R = ...
    SimscapeParams.BodyRadius;

H = ...
    SimscapeParams.BodyHeight;


SimscapeParams.BodySurfacePoints = [ ...

     R,             0,             -H/2;
     R/sqrt(2),     R/sqrt(2),     -H/2;
     0,             R,             -H/2;
    -R/sqrt(2),     R/sqrt(2),     -H/2;
    -R,             0,             -H/2;
    -R/sqrt(2),    -R/sqrt(2),     -H/2;
     0,            -R,             -H/2;
     R/sqrt(2),    -R/sqrt(2),     -H/2;

     R,             0,             -H/4;
     R/sqrt(2),     R/sqrt(2),     -H/4;
     0,             R,             -H/4;
    -R/sqrt(2),     R/sqrt(2),     -H/4;
    -R,             0,             -H/4;
    -R/sqrt(2),    -R/sqrt(2),     -H/4;
     0,            -R,             -H/4;
     R/sqrt(2),    -R/sqrt(2),     -H/4;

     R,             0,              0;
     R/sqrt(2),     R/sqrt(2),      0;
     0,             R,              0;
    -R/sqrt(2),     R/sqrt(2),      0;
    -R,             0,              0;
    -R/sqrt(2),    -R/sqrt(2),      0;
     0,            -R,              0;
     R/sqrt(2),    -R/sqrt(2),      0;

     R,             0,              H/4;
     R/sqrt(2),     R/sqrt(2),      H/4;
     0,             R,              H/4;
    -R/sqrt(2),     R/sqrt(2),      H/4;
    -R,             0,              H/4;
    -R/sqrt(2),    -R/sqrt(2),      H/4;
     0,            -R,              H/4;
     R/sqrt(2),    -R/sqrt(2),      H/4;

     R,             0,              H/2;
     R/sqrt(2),     R/sqrt(2),      H/2;
     0,             R,              H/2;
    -R/sqrt(2),     R/sqrt(2),      H/2;
    -R,             0,              H/2;
    -R/sqrt(2),    -R/sqrt(2),      H/2;
     0,            -R,              H/2;
     R/sqrt(2),    -R/sqrt(2),      H/2 ...
];


%% ------------------------------------------------------------------------
% Motor / motor-holder surface points
%
% Cylindrical surface point cloud used by the Simscape model.
% -------------------------------------------------------------------------

R_m = ...
    SimscapeParams.MotorRadius;

H_m = ...
    SimscapeParams.MotorHeight;


SimscapeParams.MotorSurfacePoints = [ ...

     R_m,             0,             -H_m/2;
     R_m/sqrt(2),     R_m/sqrt(2),   -H_m/2;
     0,               R_m,           -H_m/2;
    -R_m/sqrt(2),     R_m/sqrt(2),   -H_m/2;
    -R_m,             0,             -H_m/2;
    -R_m/sqrt(2),    -R_m/sqrt(2),   -H_m/2;
     0,              -R_m,           -H_m/2;
     R_m/sqrt(2),    -R_m/sqrt(2),   -H_m/2;

     R_m,             0,              0;
     R_m/sqrt(2),     R_m/sqrt(2),    0;
     0,               R_m,            0;
    -R_m/sqrt(2),     R_m/sqrt(2),    0;
    -R_m,             0,              0;
    -R_m/sqrt(2),    -R_m/sqrt(2),    0;
     0,              -R_m,            0;
     R_m/sqrt(2),    -R_m/sqrt(2),    0;

     R_m,             0,              H_m/2;
     R_m/sqrt(2),     R_m/sqrt(2),    H_m/2;
     0,               R_m,            H_m/2;
    -R_m/sqrt(2),     R_m/sqrt(2),    H_m/2;
    -R_m,             0,              H_m/2;
    -R_m/sqrt(2),    -R_m/sqrt(2),    H_m/2;
     0,              -R_m,            H_m/2;
     R_m/sqrt(2),    -R_m/sqrt(2),    H_m/2 ...
];

%% ------------------------------------------------------------------------
% Top payload surface points
% -------------------------------------------------------------------------

if Config.TopPayload.VariantValue == 1

    R = Config.TopPayload.Geometry.Radius;
    H = Config.TopPayload.Geometry.Height;

    % Eight points around the circumference
    theta = (0:7) * pi/4;

    % Two rows: top and bottom
    z = -H/2;

    [TH, Z] = meshgrid(theta, z);

    SimscapeParams.TopPayloadSurfacePoints = [ ...
        R*cos(TH(:)), ...
        R*sin(TH(:)), ...
        Z(:) ...
    ];

else

    SimscapeParams.TopPayloadSurfacePoints = zeros(0,3);

end
%% ------------------------------------------------------------------------
% Sanity warnings
% -------------------------------------------------------------------------

if SimscapeParams.MotorEfficiencyMeasurementTorque > ...
        SimscapeParams.MotorMaxTorque

    warning( ...
        "Motor efficiency measurement torque (%.3f N*m) exceeds " + ...
        "configured maximum motor torque (%.3f N*m). " + ...
        "These legacy Simscape values should be verified.", ...
        SimscapeParams.MotorEfficiencyMeasurementTorque, ...
        SimscapeParams.MotorMaxTorque);

end


%% ------------------------------------------------------------------------
% Summary
% -------------------------------------------------------------------------

fprintf("\n");
fprintf("========================================\n");
fprintf(" Simscape Parameters\n");
fprintf("========================================\n");


%% Body

fprintf(" Body radius       : %.4f m\n", ...
    SimscapeParams.BodyRadius);

fprintf(" Body height       : %.4f m\n", ...
    SimscapeParams.BodyHeight);

fprintf(" Body mass         : %.4f kg\n", ...
    SimscapeParams.BodyMass);

fprintf(" Body Cd           : %.4f\n", ...
    SimscapeParams.BodyCd);


%% Arms

fprintf("\n Arm geometry:\n");

fprintf("   Length          : %.4f m\n", ...
    SimscapeParams.ArmLength);

fprintf("   Radius          : %.4f m\n", ...
    SimscapeParams.ArmRadius);

fprintf("   Mass            : %.4f kg\n", ...
    SimscapeParams.ArmMass);

fprintf("   Rotation angle  : %.1f deg\n", ...
    SimscapeParams.ArmRotationAngle);

fprintf("   Cd              : %.4f\n", ...
    SimscapeParams.ArmCd);


%% Motor / holder

fprintf("\n Motor / holder:\n");

fprintf("   Radius          : %.4f m\n", ...
    SimscapeParams.MotorRadius);

fprintf("   Height          : %.4f m\n", ...
    SimscapeParams.MotorHeight);

fprintf("   Mass            : %.4f kg\n", ...
    SimscapeParams.MotorMass);

fprintf("   Holder Cd       : %.4f\n", ...
    SimscapeParams.MotorHolderCd);

fprintf("   Max torque      : %.4f N*m\n", ...
    SimscapeParams.MotorMaxTorque);

fprintf("   Max power       : %.1f W\n", ...
    SimscapeParams.MotorMaxPower);

fprintf("   Kp              : %.6f\n", ...
    SimscapeParams.MotorKp);

fprintf("   Ki              : %.6f\n", ...
    SimscapeParams.MotorKi);


%% Propeller

fprintf("\n Propeller:\n");

fprintf("   Diameter        : %.4f m\n", ...
    SimscapeParams.PropellerDiameter);

fprintf("   Mass            : %.5f kg\n", ...
    SimscapeParams.PropellerMass);

fprintf("   CW geometry     : %s\n", ...
    SimscapeParams.PropellerGeometryCW);

fprintf("   CCW geometry    : %s\n", ...
    SimscapeParams.PropellerGeometryCCW);

fprintf("   KT              : %.6e\n", ...
    SimscapeParams.PropellerKT);

fprintf("   KP              : %.6e\n", ...
    SimscapeParams.PropellerKP);


%% Command scaling

fprintf("\n Command scaling:\n");

fprintf("   Max RPM         : %.0f rpm\n", ...
    SimscapeParams.CommandMaxRPM);

fprintf("   Max omega       : %.2f rad/s\n", ...
    SimscapeParams.CommandMaxOmega);


fprintf("========================================\n\n");

end