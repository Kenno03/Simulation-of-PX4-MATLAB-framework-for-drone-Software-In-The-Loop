function SimParams = build_simulation_parameters(Config, Vehicle)
%BUILD_SIMULATION_PARAMETERS
% Common physical and actuator parameters used by the simulation models.

SimParams = struct();

%% ------------------------------------------------------------------------
% Environment
% -------------------------------------------------------------------------

SimParams.Gravity = -9.82;       % [m/s^2], ENU convention
SimParams.g       = abs(SimParams.Gravity);

SimParams.ReferenceDensity = ...
    Config.Drone.Aero.Rho;

% Home position [lat deg, lon deg, alt m]. Used by the GNSS block (ReferenceLocation) and for the
% magnetometer: the IMU field is the World Magnetic Model at this position and today's date, so the
% simulated magnetometer agrees with the declination PX4's EKF looks up for the reported GPS position.
% Change it here to fly anywhere on Earth.
SimParams.HomeLLA = [55.785, 12.521, 51];

[SimParams.MagneticFieldNED, SimParams.MagneticFieldInfo] = ...
    earth_magnetic_field(SimParams.HomeLLA);   % [uT] NED


%% ------------------------------------------------------------------------
% Simplified aerodynamic model
% -------------------------------------------------------------------------

SimParams.Cd = ...
    Config.Drone.Aero.Cd;

SimParams.CdRot = ...
    Config.Drone.Aero.CdRot;


%% ------------------------------------------------------------------------
% Vehicle mass properties
% -------------------------------------------------------------------------

SimParams.TotalMass = ...
    Vehicle.TotalMass;

SimParams.COM = ...
    Vehicle.COM;

SimParams.Inertia = ...
    Vehicle.Inertia;

SimParams.RotorPositions = ...
    Vehicle.RotorPositions;


%% ------------------------------------------------------------------------
% Propulsion coefficients
% -------------------------------------------------------------------------

SimParams.kT = ...
    Config.Propulsion.kT;

SimParams.kQ = ...
    Config.Propulsion.kQ;

SimParams.kT_Source = ...
    Config.Propulsion.kT_Source;

SimParams.kQ_Source = ...
    Config.Propulsion.kQ_Source;


%% ------------------------------------------------------------------------
% Simplified motor dynamics
% -------------------------------------------------------------------------

SimParams.MotorTimeConstant = ...
    Config.Motor.Time_Constant;


%% ------------------------------------------------------------------------
% Normalized actuator command scaling
%
% u = 1 corresponds to CommandMaxOmega.
% -------------------------------------------------------------------------

SimParams.CommandMaxRPM = ...
    Config.Propulsion.CommandMaxRPM;

SimParams.CommandMaxOmega = ...
    Config.Propulsion.CommandMaxOmega;

SimParams.CommandMaxThrustPerMotor = ...
    Config.Propulsion.CommandMaxThrust;

if ~isfinite(SimParams.CommandMaxOmega)

    error( ...
        "No normalized actuator command scale is defined for '%s'.", ...
        Config.Propulsion.Name);

end


%% ------------------------------------------------------------------------
% Physical operating limits
%
% These may legitimately be unknown.
% -------------------------------------------------------------------------

SimParams.PhysicalMaxRPM = ...
    Config.Propulsion.PhysicalMaxRPM;

SimParams.PhysicalMaxOmega = ...
    Config.Propulsion.PhysicalMaxOmega;

SimParams.PhysicalMaxThrustPerMotor = ...
    Config.Propulsion.PhysicalMaxThrust;


%% ------------------------------------------------------------------------
% Compatibility aliases
%
% Existing blocks may still use these names.
% Semantically these represent COMMAND limits, not measured physical limits.
% -------------------------------------------------------------------------

SimParams.MaxRPM = ...
    SimParams.CommandMaxRPM;

SimParams.MaxOmega = ...
    SimParams.CommandMaxOmega;

SimParams.MaxThrustPerMotor = ...
    SimParams.CommandMaxThrustPerMotor;


%% ------------------------------------------------------------------------
% Hover conditions
% -------------------------------------------------------------------------

SimParams.HoverThrust = ...
    SimParams.TotalMass * SimParams.g;

SimParams.HoverThrustPerMotor = ...
    SimParams.HoverThrust / 4;


%% ------------------------------------------------------------------------
% Rotor geometry
% -------------------------------------------------------------------------

r_rot = ...
    SimParams.RotorPositions;

rotorArmLengths = ...
    sqrt(r_rot(:,1).^2 + r_rot(:,2).^2);

SimParams.ArmLength = ...
    mean(rotorArmLengths);

SimParams.RotorHeight = ...
    mean(r_rot(:,3));

SimParams.RotorRadius = ...
    Config.Propeller.Radius;


%% ------------------------------------------------------------------------
% Rotor moment arms
% -------------------------------------------------------------------------

SimParams.RollLeverArm = ...
    mean(abs(r_rot(:,2)));

SimParams.PitchLeverArm = ...
    mean(abs(r_rot(:,1)));


%% ------------------------------------------------------------------------
% Aerospace Blockset Multirotor coefficients
%
% T = CT*rho*pi*R^4*omega^2
% Q = CQ*rho*pi*R^5*omega^2
% -------------------------------------------------------------------------

rho = ...
    SimParams.ReferenceDensity;

R = ...
    SimParams.RotorRadius;

SimParams.CT = ...
    SimParams.kT / ...
    (rho*pi*R^4);

SimParams.CQ = ...
    SimParams.kQ / ...
    (rho*pi*R^5);


%% ------------------------------------------------------------------------
% Yaw torque / thrust ratio
% -------------------------------------------------------------------------

SimParams.KM_Magnitude = ...
    SimParams.kQ / SimParams.kT;


%% ------------------------------------------------------------------------
% Command-space thrust and torque limits
% -------------------------------------------------------------------------

SimParams.MinThrustPerMotor = 0;
SimParams.MinTotalThrust    = 0;

SimParams.CommandMaxTotalThrust = ...
    4 * SimParams.CommandMaxThrustPerMotor;

% Existing Simulink saturation blocks use this name.
SimParams.MaxTotalThrust = ...
    SimParams.CommandMaxTotalThrust;

SimParams.HoverThrustFraction = ...
    SimParams.HoverThrust / ...
    SimParams.CommandMaxTotalThrust;


%% Roll

SimParams.MaxTauX = ...
    2 * ...
    SimParams.RollLeverArm * ...
    SimParams.CommandMaxThrustPerMotor;

SimParams.MinTauX = ...
    -SimParams.MaxTauX;


%% Pitch

SimParams.MaxTauY = ...
    2 * ...
    SimParams.PitchLeverArm * ...
    SimParams.CommandMaxThrustPerMotor;

SimParams.MinTauY = ...
    -SimParams.MaxTauY;


%% Yaw

SimParams.MaxTauZ = ...
    2 * ...
    SimParams.KM_Magnitude * ...
    SimParams.CommandMaxThrustPerMotor;

SimParams.MinTauZ = ...
    -SimParams.MaxTauZ;


%% ------------------------------------------------------------------------
% Summary
% -------------------------------------------------------------------------

fprintf("\n");
fprintf("========================================\n");
fprintf(" Simulation Parameters\n");
fprintf("========================================\n");

fprintf(" Total mass        : %.4f kg\n", ...
    SimParams.TotalMass);

fprintf(" Gravity           : %.3f m/s^2\n", ...
    SimParams.Gravity);

fprintf(" Air density       : %.3f kg/m^3\n", ...
    SimParams.ReferenceDensity);

fprintf(" Home (lat,lon,alt): %.5f, %.5f, %.1f m\n", ...
    SimParams.HomeLLA);

fprintf(" Magnetic field    : %s, decl %.2f deg, incl %.2f deg, %.1f uT\n", ...
    SimParams.MagneticFieldInfo.Model, ...
    SimParams.MagneticFieldInfo.Declination_deg, ...
    SimParams.MagneticFieldInfo.Inclination_deg, ...
    SimParams.MagneticFieldInfo.Strength_uT);

fprintf(" Cd                : [%.3f %.3f %.3f]\n", ...
    SimParams.Cd);

fprintf(" CdRot             : [%.3f %.3f %.3f]\n", ...
    SimParams.CdRot);

fprintf(" Hover thrust      : %.3f N\n", ...
    SimParams.HoverThrust);

fprintf(" Hover / motor     : %.3f N\n", ...
    SimParams.HoverThrustPerMotor);

fprintf(" Arm length        : %.4f m\n", ...
    SimParams.ArmLength);

fprintf(" Rotor height      : %.4f m\n", ...
    SimParams.RotorHeight);

fprintf(" Rotor radius      : %.4f m\n", ...
    SimParams.RotorRadius);

fprintf(" kT                : %.6e (%s)\n", ...
    SimParams.kT, ...
    SimParams.kT_Source);

fprintf(" kQ                : %.6e (%s)\n", ...
    SimParams.kQ, ...
    SimParams.kQ_Source);

fprintf(" CT                : %.6e\n", ...
    SimParams.CT);

fprintf(" CQ                : %.6e\n", ...
    SimParams.CQ);

fprintf(" KM                : %.6e m\n", ...
    SimParams.KM_Magnitude);

fprintf(" Motor tau         : %.4f s\n", ...
    SimParams.MotorTimeConstant);


fprintf("\n Command scaling:\n");

fprintf("   Max RPM         : %.0f rpm\n", ...
    SimParams.CommandMaxRPM);

fprintf("   Max omega       : %.2f rad/s\n", ...
    SimParams.CommandMaxOmega);

fprintf("   Max thrust/motor: %.3f N\n", ...
    SimParams.CommandMaxThrustPerMotor);

fprintf("   Max total thrust: %.3f N\n", ...
    SimParams.CommandMaxTotalThrust);

fprintf("   Hover fraction  : %.4f\n", ...
    SimParams.HoverThrustFraction);


fprintf("\n Physical limit:\n");

if isfinite(SimParams.PhysicalMaxRPM)

    fprintf("   Max RPM         : %.0f rpm\n", ...
        SimParams.PhysicalMaxRPM);

    fprintf("   Max thrust/motor: %.3f N\n", ...
        SimParams.PhysicalMaxThrustPerMotor);

else

    fprintf("   UNKNOWN\n");

end


fprintf("\n Command torque limits:\n");

fprintf("   Max tau_x       : %.3f N*m\n", ...
    SimParams.MaxTauX);

fprintf("   Max tau_y       : %.3f N*m\n", ...
    SimParams.MaxTauY);

fprintf("   Max tau_z       : %.3f N*m\n", ...
    SimParams.MaxTauZ);

fprintf("========================================\n\n");

end