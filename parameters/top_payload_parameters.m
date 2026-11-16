function TopPayload = top_payload_parameters(TopPayloadChoice)
%TOP_PAYLOAD_PARAMETERS Catalogue of payloads mounted on top of the drone body.
%   Options = top_payload_parameters() lists the available top payloads.
%   TopPayload = top_payload_parameters(TopPayloadChoice) returns the selected payload:
%   VariantValue (plant variant), ID, Name, Mass [kg], Inertia_COM [kg*m^2], InterfaceToCOM [m],
%   Rotation, Geometry and Aero (drag). Used by load_configuration ("TopPayloadChoice" in Main.mlx).

%% Available top payloads
TopPayloadOptions = [ ...
    "None"
    "Top_Cylinder_100g"
];

%% Return options if no input
if nargin == 0
    TopPayload = TopPayloadOptions;
    return
end

%% Selected top payload
switch TopPayloadChoice

    case "None"

        TopPayload = struct();
        TopPayload.VariantValue = 0;
        %% Identification
        TopPayload.ID   = "None";
        TopPayload.Name = "No Top Payload";

        %% Mass properties
        TopPayload.Mass = 0;                    % [kg]
        TopPayload.Inertia_COM = zeros(3);      % [kg*m^2]

        %% COM relative to mounting interface
        TopPayload.InterfaceToCOM = [0 0 0];    % [m]

        %% Orientation relative to drone body
        TopPayload.Rotation = eye(3);

        %% Geometry
        TopPayload.Geometry.Type = "None";
        TopPayload.Geometry.STEP = "";


    case "Top_Cylinder_100g"

        TopPayload = struct();
        TopPayload.VariantValue = 1;

        %% Identification
        TopPayload.ID   = "Top_Cylinder_100g";
        TopPayload.Name = "Top Payload - 100 g Cylinder";

        %% Geometry: a flat cylinder on top of the body, same radius as the unit drone's body
        TopPayload.Geometry.Type   = "Cylinder";
        TopPayload.Geometry.Radius = 50e-3;     % [m] = unit drone body radius
        TopPayload.Geometry.Height = 30e-3;     % [m]
        TopPayload.Geometry.STEP   = "";

        %% Mass properties
        TopPayload.Mass = 100e-3;               % [kg]

        m = TopPayload.Mass;
        r = TopPayload.Geometry.Radius;
        h = TopPayload.Geometry.Height;

        % Solid cylinder inertia about its own COM.
        % Cylinder symmetry axis is aligned with body z.
        Ixx = (m/12) * (3*r^2 + h^2);
        Iyy = Ixx;
        Izz = (m/2) * r^2;

        TopPayload.Inertia_COM = diag([Ixx Iyy Izz]);

        %% COM relative to mounting interface
        %
        % Body convention:
        %   +x = forward
        %   +y = right
        %   +z = down
        %
        % Mounting interface is at the bottom face.
        % Payload extends upwards in -z.
        %
        % Therefore its COM lies h/2 above the interface.

        TopPayload.InterfaceToCOM = [0 0 -h/2]; % [m]

        %% Orientation relative to drone body
        TopPayload.Rotation = eye(3);

        %% Aerodynamics
        TopPayload.Aero.Cd = diag([1 1 1]);  % 3x3 drag coefficient matrix
        TopPayload.Aero.Axis = [0 0 1]; % [-] Cylinder axis

    otherwise

        error("Unknown top payload: %s", TopPayloadChoice)

end

end
