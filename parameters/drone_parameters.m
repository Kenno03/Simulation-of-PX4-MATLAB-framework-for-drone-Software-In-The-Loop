function P = drone_parameters(DroneChoice)
%DRONE_PARAMETERS Catalogue of drone frames (body and arms, without motors, propellers or payload).
%   Options = drone_parameters() lists the available frames.
%   P = drone_parameters(DroneChoice) returns the selected frame: ID, Name, body radius/height/mass,
%   arm length/radius/mass and Aero (drag coefficients and air density). Used by load_configuration;
%   compute_drone_geometry_inertia combines it with the other components into the vehicle and
%   calculates the mass, centre of mass and inertia from this geometry.

%% Available frames
DroneOptions = [ ...
    "UnitDrone"
];

%% Return options if no input
if nargin == 0
    P = DroneOptions;
    return
end

%% Selected frame
switch DroneChoice

    case "UnitDrone"

        % Generic quadcopter with round numbers ("unit drone"). With the 2212 motors and the
        % 10x4.5 propellers of the catalogues the drone weighs 1.000 kg without payload:
        %   body 0.640 kg + 4 arms x 0.025 kg + 4 motors x 0.056 kg + 4 propellers x 0.009 kg.
        % The inertia follows from this geometry (compute_drone_geometry_inertia); with the
        % cylindrical body and four identical arms it is symmetric in roll and pitch (Ixx = Iyy).

        P = struct();

        %% ----------------------------------------------------------------
        % Identification
        % -----------------------------------------------------------------

        P.ID   = "UnitDrone";
        P.Name = "Unit drone (generic 1 kg quadcopter)";


        %% ----------------------------------------------------------------
        % Body geometry (solid cylinder, symmetry axis = body z)
        % -----------------------------------------------------------------

        P.Body_Radius = 50e-3;        % [m]
        P.Body_Height = 100e-3;       % [m]
        P.Body_Mass   = 0.640;        % [kg]


        %% ----------------------------------------------------------------
        % Arm geometry (four identical arms in X configuration)
        % -----------------------------------------------------------------

        P.Arm_Length = 200e-3;        % [m]
        P.Arm_Radius = 8e-3;          % [m]
        P.Arm_Mass   = 25e-3;         % [kg]


        %% ----------------------------------------------------------------
        % Aerodynamics
        % -----------------------------------------------------------------

        % Reference atmospheric density used by the current model
        P.Aero.Rho = 1.225;           % [kg/m^3]

        % Simplified translational drag model
        P.Aero.Cd = [0.05 0.05 0.01];

        % Scaling of simplified rotational damping model
        P.Aero.CdRot = [1 1 1];


        %% Detailed Simscape aerodynamic properties

        % Main body
        P.Aero.Body.Cd   = 1.0;
        P.Aero.Body.Axis = [0 0 1];

        % Arms
        P.Aero.Arm.Cd   = 1.0;
        P.Aero.Arm.Axis = [0 0 1];

        % Motor holders
        P.Aero.MotorHolder.Cd   = 1.0;
        P.Aero.MotorHolder.Axis = [0 0 1];


    otherwise

        error("Unknown drone configuration: %s", DroneChoice)

end

end
