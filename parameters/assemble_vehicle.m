function Vehicle = assemble_vehicle(Config)
%ASSEMBLE_VEHICLE
% Assemble the selected drone, propulsion hardware and payloads into the
% final physical vehicle parameters.
%
% Coordinate convention:
%
%   +x = forward
%   +y = right
%   +z = down
%
% Payloads are mounted centrally on the drone:
%
%   Top payload:
%       mounted at the top surface of the drone body
%
%   Bottom payload:
%       mounted at the bottom surface of the drone body
%
% Each payload defines its own COM relative to its mounting interface
% through:
%
%   Payload.InterfaceToCOM
%
% This allows arbitrary payload geometry while keeping the mounting
% location standardized.


%% ------------------------------------------------------------------------
% Base drone
% -------------------------------------------------------------------------

P = Config.Drone;

%% Apply selected motor physical properties

P.Motor_Mass = ...
    Config.Motor.Mass;

P.Motor_Radius = ...
    Config.Motor.Radius;

P.Motor_Height = ...
    Config.Motor.Height;

%% ------------------------------------------------------------------------
% Apply selected propeller physical properties
% -------------------------------------------------------------------------

P.Propeller_Mass = ...
    Config.Propeller.Mass;

P.Propeller_Disk_Diameter = ...
    Config.Propeller.Diameter;


%% ------------------------------------------------------------------------
% Compute base drone geometry, mass, COM and inertia
% -------------------------------------------------------------------------

P = compute_drone_geometry_inertia( ...
    P, ...
    "body_plus_parts");


M_base = ...
    P.Derived.Total_Mass;

r_base = ...
    P.Derived.COM(:);

J_base = ...
    P.Derived.Inertia_COM;


%% ------------------------------------------------------------------------
% Payload mounting interfaces
% -------------------------------------------------------------------------
%
% Payloads are mounted directly above or below the drone body.
%
% +z points downward.
%
% Therefore:
%
%   Top interface    -> negative z
%   Bottom interface -> positive z

TopInterface = [ ...
    0;
    0;
    -P.Body_Height/2 ...
];

BottomInterface = [ ...
    0;
    0;
    +P.Body_Height/2 ...
];


%% ------------------------------------------------------------------------
% Payload COM positions
% -------------------------------------------------------------------------
%
% InterfaceToCOM is defined in drone body coordinates.

r_top = ...
    TopInterface + ...
    Config.TopPayload.InterfaceToCOM(:);

r_bottom = ...
    BottomInterface + ...
    Config.BottomPayload.InterfaceToCOM(:);


%% ------------------------------------------------------------------------
% Payload lists
% -------------------------------------------------------------------------

Payloads = { ...
    Config.TopPayload, ...
    Config.BottomPayload ...
};

PayloadPositions = { ...
    r_top, ...
    r_bottom ...
};


%% ------------------------------------------------------------------------
% Total mass
% -------------------------------------------------------------------------

M_total = M_base;

for k = 1:numel(Payloads)

    Payload = Payloads{k};

    M_total = ...
        M_total + Payload.Mass;

end


%% ------------------------------------------------------------------------
% Combined center of mass
% -------------------------------------------------------------------------

COM_numerator = ...
    M_base * r_base;


for k = 1:numel(Payloads)

    Payload = Payloads{k};

    m = Payload.Mass;

    if m <= 0
        continue
    end

    r_payload = ...
        PayloadPositions{k};

    COM_numerator = ...
        COM_numerator + ...
        m*r_payload;

end


r_COM = ...
    COM_numerator / M_total;


%% ------------------------------------------------------------------------
% Combined inertia about final vehicle COM
% -------------------------------------------------------------------------

% Shift base-drone inertia from the base-drone COM
% to the final assembled vehicle COM.

d_base = ...
    r_base - r_COM;

J_total = ...
    J_base + ...
    M_base * ...
    ( ...
        dot(d_base,d_base)*eye(3) ...
        - d_base*d_base.' ...
    );


%% ------------------------------------------------------------------------
% Add payload inertia contributions
% -------------------------------------------------------------------------

for k = 1:numel(Payloads)

    Payload = Payloads{k};

    m = Payload.Mass;

    if m <= 0
        continue
    end


    %% Payload COM position

    r_payload = ...
        PayloadPositions{k};


    %% Payload orientation

    R_payload = ...
        Payload.Rotation;


    %% Rotate payload inertia into drone body coordinates

    J_payload_body = ...
        R_payload * ...
        Payload.Inertia_COM * ...
        R_payload.';


    %% Shift payload inertia to final vehicle COM

    d = ...
        r_payload - r_COM;

    J_total = ...
        J_total + ...
        J_payload_body + ...
        m * ...
        ( ...
            dot(d,d)*eye(3) ...
            - d*d.' ...
        );

end


%% ------------------------------------------------------------------------
% Rotor positions relative to final vehicle COM
% -------------------------------------------------------------------------

rotorPositionsOrigin = ...
    P.Derived.Rotor_Positions_Origin;

rotorPositionsCOM = ...
    rotorPositionsOrigin - r_COM.';


%% ------------------------------------------------------------------------
% Output
% -------------------------------------------------------------------------

Vehicle = struct();


%% Original / derived drone parameters

Vehicle.Parameters = P;


%% Final mass properties

Vehicle.TotalMass = ...
    M_total;

Vehicle.COM = ...
    r_COM.';

Vehicle.Inertia = ...
    J_total;


%% Rotor geometry

Vehicle.RotorPositions = ...
    rotorPositionsCOM;


%% Base vehicle

Vehicle.BaseMass = ...
    M_base;


%% Payload information

Vehicle.TopPayloadMass = ...
    Config.TopPayload.Mass;

Vehicle.BottomPayloadMass = ...
    Config.BottomPayload.Mass;

Vehicle.TopPayloadCOM = ...
    r_top.';

Vehicle.BottomPayloadCOM = ...
    r_bottom.';

Vehicle.TopInterface = ...
    TopInterface.';

Vehicle.BottomInterface = ...
    BottomInterface.';


%% ------------------------------------------------------------------------
% Summary
% -------------------------------------------------------------------------

fprintf("\n");
fprintf("========================================\n");
fprintf(" Assembled Vehicle\n");
fprintf("========================================\n");

fprintf(" Base mass       : %.4f kg\n", ...
    M_base);

fprintf(" Top payload     : %.4f kg\n", ...
    Config.TopPayload.Mass);

fprintf(" Bottom payload  : %.4f kg\n", ...
    Config.BottomPayload.Mass);

fprintf(" Total mass      : %.4f kg\n", ...
    M_total);

fprintf(" COM [x y z]     : [%+.4f %+.4f %+.4f] m\n", ...
    r_COM);

fprintf("========================================\n\n");

end