function BottomPayload = bottom_payload_parameters(BottomPayloadChoice)
%BOTTOM_PAYLOAD_PARAMETERS Catalogue of payloads mounted under the drone body.
%   Options = bottom_payload_parameters() lists the available bottom payloads.
%   BottomPayload = bottom_payload_parameters(BottomPayloadChoice) returns the selected payload:
%   ID, Name, Mass [kg], Inertia_COM [kg*m^2], InterfaceToCOM [m], Rotation and Geometry.
%   Used by load_configuration (Main.mlx selection "BottomPayloadChoice").

%% Available bottom payloads
BottomPayloadOptions = [ ...
    "None"
];

%% Return options if no input
if nargin == 0
    BottomPayload = BottomPayloadOptions;
    return
end

%% Selected bottom payload
switch BottomPayloadChoice

    case "None"

        BottomPayload = struct();

        %% Identification
        BottomPayload.ID   = "None";
        BottomPayload.Name = "No Bottom Payload";

        %% Mass properties
        BottomPayload.Mass = 0;                       % [kg]

        BottomPayload.Inertia_COM = zeros(3);         % [kg*m^2]

        %% COM relative to mounting interface
        %
        % Bottom payload normally extends in +z.

        BottomPayload.InterfaceToCOM = [0 0 0];       % [m]

        %% Orientation relative to drone body
        BottomPayload.Rotation = eye(3);

        %% Geometry
        BottomPayload.Geometry.Type = "None";
        BottomPayload.Geometry.STEP = "";

    otherwise

        error("Unknown bottom payload: %s", BottomPayloadChoice)

end

end