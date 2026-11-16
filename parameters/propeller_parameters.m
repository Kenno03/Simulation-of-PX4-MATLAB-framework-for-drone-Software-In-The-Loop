function Propeller = propeller_parameters(PropellerChoice)
%PROPELLER_PARAMETERS Catalogue of propellers.
%   Options = propeller_parameters() lists the available propellers.
%   Propeller = propeller_parameters(PropellerChoice) returns the selected propeller: ID, Name,
%   diameter/radius, pitch, number of blades, mass (and its source), Geometry (CAD file for the
%   Simscape solid), BEMT (thrust and torque data, Propeller/*.mat), bore diameter and Reference.
%   Used by load_configuration; get_propeller_coefficients turns the BEMT data into kT and kQ.

%% ------------------------------------------------------------------------
% Available propellers
% -------------------------------------------------------------------------

PropellerOptions = [ ...
    "2Blade_8in"
    "2Blade_10x4_5in"
];


%% ------------------------------------------------------------------------
% Return options if no input
% -------------------------------------------------------------------------

if nargin == 0

    Propeller = PropellerOptions;
    return

end


%% ------------------------------------------------------------------------
% Project paths
%
% propeller_parameters.m is located directly in the project root.
%
% Use absolute paths for CAD files so Simscape File Solid blocks can
% resolve them independently of the current model/subsystem location.
% -------------------------------------------------------------------------

thisFile = ...
    mfilename('fullpath');

projectRoot = ...
    fileparts(fileparts(thisFile));   % this file is in <project>/parameters

stepFolder = ...
    fullfile(projectRoot, 'step files');


%% ------------------------------------------------------------------------
% Selected propeller
% -------------------------------------------------------------------------

switch PropellerChoice


    case "2Blade_8in"

        Propeller = struct();


        %% -------------------------------------------------------------
        % Identification
        % --------------------------------------------------------------

        Propeller.ID = ...
            "2Blade_8in";

        Propeller.Name = ...
            "2 Blade 8 inch";


        %% -------------------------------------------------------------
        % Geometry
        % --------------------------------------------------------------

        Propeller.Diameter = ...
            8 * 0.0254;                    % [m]

        Propeller.Radius = ...
            Propeller.Diameter / 2;

        Propeller.Pitch_in = ...
            4.3;                           % [inch]

        Propeller.Pitch = ...
            Propeller.Pitch_in * 0.0254;   % [m]

        Propeller.NumberOfBlades = ...
            2;


        %% -------------------------------------------------------------
        % Mass
        % --------------------------------------------------------------

        Propeller.Mass = ...
            7.79e-3;                       % [kg]

        Propeller.MassSource = ...
            "Existing model value";


        %% -------------------------------------------------------------
        % CAD geometry
        % --------------------------------------------------------------

        Propeller.Geometry.CW = ...
            fullfile( ...
                stepFolder, ...
                'CW 2Blade_8in.stp');

        Propeller.Geometry.CCW = ...
            fullfile( ...
                stepFolder, ...
                'CCW 2Blade_8in.stp');


        %% -------------------------------------------------------------
        % BEMT approximation
        % --------------------------------------------------------------

        Propeller.BEMT.NumberOfBlades = ...
            2;

        Propeller.BEMT.Chord = ...
            0.028;                         % [m], approximate

        Propeller.BEMT.LiftCurveSlope = ...
            5.7;                           % [1/rad]

        Propeller.BEMT.MeanDragCoefficient = ...
            0.012;

        Propeller.BEMT.HingeOffset = ...
            0;

        Propeller.BEMT.TwistType = ...
            "Ideal";

        Propeller.BEMT.ThetaTip = ...
            atan( ...
                Propeller.Pitch / ...
                (2*pi*Propeller.Radius));

        Propeller.BEMT.NumberOfElements = ...
            100;

        Propeller.BEMT.ReferenceDensity = ...
            1.225;                         % [kg/m^3]



    case "2Blade_10x4_5in"

        Propeller = struct();


        %% -------------------------------------------------------------
        % Identification
        % --------------------------------------------------------------

        Propeller.ID = ...
            "2Blade_10x4_5in";

        Propeller.Name = ...
            "2 Blade 10x4.5 inch";


        %% -------------------------------------------------------------
        % Geometry
        % --------------------------------------------------------------

        Propeller.Diameter = ...
            10 * 0.0254;                   % [m]

        Propeller.Radius = ...
            Propeller.Diameter / 2;

        Propeller.Pitch_in = ...
            4.5;                           % [inch]

        Propeller.Pitch = ...
            Propeller.Pitch_in * 0.0254;   % [m]

        Propeller.NumberOfBlades = ...
            2;

        Propeller.BoreDiameter = ...
            6.3e-3;                        % [m]


        %% -------------------------------------------------------------
        % Mass
        %
        % Typical mass of a 10x4.5 carbon-fibre multirotor propeller.
        % --------------------------------------------------------------

        Propeller.Mass = ...
            9.0e-3;                        % [kg]

        Propeller.MassSource = ...
            "Typical 10x4.5 carbon-fibre propeller";


        %% -------------------------------------------------------------
        % CAD geometry
        % --------------------------------------------------------------

        Propeller.Geometry.CW = ...
            fullfile( ...
                stepFolder, ...
                'CW 2Blade_10x4.5in.stp');

        Propeller.Geometry.CCW = ...
            fullfile( ...
                stepFolder, ...
                'CCW 2Blade_10x4.5in.stp');


        %% -------------------------------------------------------------
        % Static aerodynamic reference
        %
        % Initial proxy based on APC 10x4.5 MR data.
        %
        % Standard propeller coefficients:
        %
        %   T = KT * rho * n^2 * D^4
        %
        %   P = KP * rho * n^3 * D^5
        %
        % These are reference data for a standard 10x4.5 propeller, not measurements.
        % --------------------------------------------------------------

        Propeller.Reference.KT_Prop = ...
            0.1126;

        Propeller.Reference.KP_Prop = ...
            0.0430;

        Propeller.Reference.Source = ...
            "APC 10x4.5 MR static-data proxy";


        %% -------------------------------------------------------------
        % Recommended rotational-speed limit
        %
        % RPM * Diameter[in] <= 105000
        %
        % For a 10 inch propeller:
        %
        %   RPM <= 10500
        % --------------------------------------------------------------

        Propeller.Reference.RPMDiameterLimit = ...
            105000;                         % [rpm*inch]

        Propeller.Reference.MaxRecommendedRPM = ...
            Propeller.Reference.RPMDiameterLimit / ...
            10;                             % [rpm]


        %% -------------------------------------------------------------
        % BEMT approximation
        %
        % Retained as a secondary model.
        %
        % Mean chord is provisional until actual blade geometry is
        % identified.
        % --------------------------------------------------------------

        Propeller.BEMT.NumberOfBlades = ...
            2;

        Propeller.BEMT.Chord = ...
            0.035;                          % [m], provisional

        Propeller.BEMT.LiftCurveSlope = ...
            5.7;                            % [1/rad]

        Propeller.BEMT.MeanDragCoefficient = ...
            0.012;

        Propeller.BEMT.HingeOffset = ...
            0;

        Propeller.BEMT.TwistType = ...
            "Ideal";

        Propeller.BEMT.ThetaTip = ...
            atan( ...
                Propeller.Pitch / ...
                (2*pi*Propeller.Radius));

        Propeller.BEMT.NumberOfElements = ...
            100;

        Propeller.BEMT.ReferenceDensity = ...
            1.225;                          % [kg/m^3]


    otherwise

        error( ...
            "Unknown propeller: %s", ...
            PropellerChoice);

end


%% ------------------------------------------------------------------------
% Validate CAD files
% -------------------------------------------------------------------------

if ~isfile(Propeller.Geometry.CW)

    error( ...
        "CW propeller CAD file does not exist:\n%s", ...
        Propeller.Geometry.CW);

end


if ~isfile(Propeller.Geometry.CCW)

    error( ...
        "CCW propeller CAD file does not exist:\n%s", ...
        Propeller.Geometry.CCW);

end

end