function Propulsion = propulsion_parameters(Motor, Propeller)
%PROPULSION_PARAMETERS
% Determines propulsion parameters for a selected motor/propeller pair.
%
% PhysicalMax...:
%   Measured / validated physical operating limit.
%
% CommandMax...:
%   Rotor operating point represented by normalized actuator command u = 1.
%
% These are deliberately kept separate.

Propulsion = struct();


%% ------------------------------------------------------------------------
% Identification
% -------------------------------------------------------------------------

pairID = ...
    Motor.ID + "__" + Propeller.ID;

Propulsion.ID = ...
    pairID;

Propulsion.Name = ...
    Motor.Name + " + " + Propeller.Name;


%% ------------------------------------------------------------------------
% Selected motor / propeller combination
% -------------------------------------------------------------------------

switch pairID

    case "2212_920KV__2Blade_10x4_5in"

        %% -------------------------------------------------------------
        % 10x4.5 multirotor aerodynamic proxy
        %
        % Standard propeller relations:
        %
        %   T = KT * rho * n^2 * D^4
        %
        %   P = KP * rho * n^3 * D^5
        %
        % with:
        %
        %   n = omega / (2*pi)
        %
        % Therefore:
        %
        %   T = kT * omega^2
        %
        %   Q = kQ * omega^2
        %
        % where:
        %
        %   kT = KT*rho*D^4/(4*pi^2)
        %
        %   kQ = KP*rho*D^5/(8*pi^3)
        % --------------------------------------------------------------

        rho = ...
            Propeller.BEMT.ReferenceDensity;

        D = ...
            Propeller.Diameter;

        KT = ...
            Propeller.Reference.KT_Prop;

        KP = ...
            Propeller.Reference.KP_Prop;


        Propulsion.kT = ...
            KT * rho * D^4 / ...
            (4*pi^2);

        Propulsion.kQ = ...
            KP * rho * D^5 / ...
            (8*pi^3);

        Propulsion.kT_Source = ...
            "APC 10x4.5 MR aerodynamic proxy";

        Propulsion.kQ_Source = ...
            "APC 10x4.5 MR aerodynamic proxy";


        %% -------------------------------------------------------------
        % Maximum rotor speed
        %
        % Estimate for a 920 KV motor on a 3S battery (11.1 V) driving a
        % 10x4.5 propeller: no-load speed 920*11.1 = 10200 rpm, about
        % 8000 rpm under propeller load (about 1 kg thrust per motor).
        % Not bench tested.
        % --------------------------------------------------------------

        Propulsion.PhysicalMaxRPM = ...
            8000;

        Propulsion.PhysicalMaxOmega = ...
            Propulsion.PhysicalMaxRPM * 2*pi/60;

        Propulsion.PhysicalMaxThrust = ...
            NaN;

        Propulsion.PhysicalLimitSource = ...
            "Estimate: 920 KV x 11.1 V under 10x4.5 propeller load (not bench tested)";


        %% -------------------------------------------------------------
        % Normalized command scaling
        %
        % Command u = 1 corresponds to the estimated maximum loaded speed.
        % --------------------------------------------------------------

        Propulsion.CommandMaxRPM = ...
            Propulsion.PhysicalMaxRPM;

        Propulsion.CommandMaxOmega = ...
            Propulsion.CommandMaxRPM * ...
            2*pi/60;

        Propulsion.CommandScaleSource = ...
            "Estimated maximum loaded speed (920 KV, 3S, 10x4.5)";


    case "2212_1400KV__2Blade_8in"

        %% -------------------------------------------------------------
        % 1400 KV + 8 inch propeller (BEMT coefficients)
        % --------------------------------------------------------------

        BEMT = ...
            get_propeller_coefficients(Propeller);

        Propulsion.BEMT = ...
            BEMT;

        Propulsion.kT = ...
            BEMT.kT_omega;

        Propulsion.kQ = ...
            BEMT.kQ_omega;

        Propulsion.kT_Source = ...
            "BEMT";

        Propulsion.kQ_Source = ...
            "BEMT";


        %% Physical maximum currently unknown

        Propulsion.PhysicalMaxRPM = ...
            NaN;

        Propulsion.PhysicalMaxOmega = ...
            NaN;

        Propulsion.PhysicalMaxThrust = ...
            NaN;

        Propulsion.PhysicalLimitSource = ...
            "Unknown";


        %% Command scaling: estimated loaded speed of 1400 KV on 3S (11.1 V)

        Propulsion.CommandMaxRPM = ...
            13500;

        Propulsion.CommandMaxOmega = ...
            Propulsion.CommandMaxRPM * ...
            2*pi/60;

        Propulsion.CommandScaleSource = ...
            "Estimated maximum loaded speed (1400 KV, 3S, 8 inch)";


    otherwise

        %% -------------------------------------------------------------
        % Generic BEMT fallback
        % --------------------------------------------------------------

        BEMT = ...
            get_propeller_coefficients(Propeller);

        Propulsion.BEMT = ...
            BEMT;

        Propulsion.kT = ...
            BEMT.kT_omega;

        Propulsion.kQ = ...
            BEMT.kQ_omega;

        Propulsion.kT_Source = ...
            "BEMT";

        Propulsion.kQ_Source = ...
            "BEMT";


        %% No validated physical maximum

        Propulsion.PhysicalMaxRPM = ...
            NaN;

        Propulsion.PhysicalMaxOmega = ...
            NaN;

        Propulsion.PhysicalMaxThrust = ...
            NaN;

        Propulsion.PhysicalLimitSource = ...
            "Unknown";


        %% No command scale known

        Propulsion.CommandMaxRPM = ...
            NaN;

        Propulsion.CommandMaxOmega = ...
            NaN;

        Propulsion.CommandScaleSource = ...
            "Unknown";

end


%% ------------------------------------------------------------------------
% Command-point derived quantities
% -------------------------------------------------------------------------

if isfinite(Propulsion.CommandMaxOmega)

    Propulsion.CommandMaxThrust = ...
        Propulsion.kT * ...
        Propulsion.CommandMaxOmega^2;

    Propulsion.CommandMaxTorque = ...
        Propulsion.kQ * ...
        Propulsion.CommandMaxOmega^2;

else

    Propulsion.CommandMaxThrust = ...
        NaN;

    Propulsion.CommandMaxTorque = ...
        NaN;

end


%% ------------------------------------------------------------------------
% Motor torque estimate
%
% Ideal:
%
%   tau = kt * I
% -------------------------------------------------------------------------

Propulsion.MotorTorqueLimit = ...
    Motor.kt * ...
    Motor.Max_Current;


if isfinite(Propulsion.CommandMaxTorque)

    Propulsion.CommandCurrentIdeal = ...
        Propulsion.CommandMaxTorque / ...
        Motor.kt;

else

    Propulsion.CommandCurrentIdeal = ...
        NaN;

end


%% ------------------------------------------------------------------------
% Torque / thrust ratio
% -------------------------------------------------------------------------

Propulsion.Torque_Thrust_Ratio = ...
    Propulsion.kQ / ...
    Propulsion.kT;


%% ------------------------------------------------------------------------
% Sanity check
% -------------------------------------------------------------------------

if isfinite(Propulsion.CommandMaxTorque) && ...
   Propulsion.CommandMaxTorque > ...
   Propulsion.MotorTorqueLimit

    warning( ...
        "Command operating point for '%s' requires %.3f N*m, " + ...
        "which exceeds the ideal motor current torque limit of %.3f N*m.", ...
        Propulsion.Name, ...
        Propulsion.CommandMaxTorque, ...
        Propulsion.MotorTorqueLimit);

end


%% ------------------------------------------------------------------------
% Summary
% -------------------------------------------------------------------------

fprintf("\n");
fprintf("========================================\n");
fprintf(" Propulsion Parameters\n");
fprintf("========================================\n");

fprintf( ...
    " Combination       : %s\n", ...
    Propulsion.Name);

fprintf( ...
    " kT                : %.6e  (%s)\n", ...
    Propulsion.kT, ...
    Propulsion.kT_Source);

fprintf( ...
    " kQ                : %.6e  (%s)\n", ...
    Propulsion.kQ, ...
    Propulsion.kQ_Source);


if isfinite(Propulsion.CommandMaxRPM)

    fprintf( ...
        " Command max RPM   : %.0f rpm\n", ...
        Propulsion.CommandMaxRPM);

    fprintf( ...
        " Command max omega : %.2f rad/s\n", ...
        Propulsion.CommandMaxOmega);

    fprintf( ...
        " Command max thrust: %.3f N\n", ...
        Propulsion.CommandMaxThrust);

    fprintf( ...
        " Command max torque: %.3f N*m\n", ...
        Propulsion.CommandMaxTorque);

    fprintf( ...
        " Ideal motor current: %.2f A\n", ...
        Propulsion.CommandCurrentIdeal);

    fprintf( ...
        " Command source    : %s\n", ...
        Propulsion.CommandScaleSource);

else

    fprintf(" Command scale     : UNKNOWN\n");

end


if isfinite(Propulsion.PhysicalMaxRPM)

    fprintf( ...
        " Physical max RPM  : %.0f rpm\n", ...
        Propulsion.PhysicalMaxRPM);

    fprintf( ...
        " Physical max thrust: %.3f N\n", ...
        Propulsion.PhysicalMaxThrust);

else

    fprintf(" Physical max      : UNKNOWN\n");

end


fprintf( ...
    " Motor torque limit: %.3f N*m\n", ...
    Propulsion.MotorTorqueLimit);

fprintf("========================================\n\n");

end