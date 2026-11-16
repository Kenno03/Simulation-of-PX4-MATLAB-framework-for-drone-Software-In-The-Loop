function Config = load_configuration( ...
    DroneChoice, PlantChoice, MotorChoice, PropellerChoice, ...
    TopPayloadChoice, BottomPayloadChoice)
%LOAD_CONFIGURATION Collect the selected drone components into one configuration struct.
%   Config = load_configuration(DroneChoice, PlantChoice, MotorChoice, PropellerChoice, ...
%       TopPayloadChoice, BottomPayloadChoice)
%   looks up each choice in its catalogue (drone_parameters, plant_parameters, motor_parameters,
%   propeller_parameters, top_payload_parameters, bottom_payload_parameters) and adds the derived
%   propulsion (propulsion_parameters) and motor-controller (motor_controller_parameters) data.
%   Fields: Drone, Plant, Motor, Propeller, TopPayload, BottomPayload, Propulsion, MotorController.
%   Called by initialization.m with the choices made in Main.mlx.

Config = struct();

%% ------------------------------------------------------------------------
% Selected components
% -------------------------------------------------------------------------

Config.Drone = ...
    drone_parameters(DroneChoice);

Config.Plant = ...
    plant_parameters(PlantChoice);

Config.Motor = ...
    motor_parameters(MotorChoice);

Config.Propeller = ...
    propeller_parameters(PropellerChoice);

Config.TopPayload = ...
    top_payload_parameters(TopPayloadChoice);

Config.BottomPayload = ...
    bottom_payload_parameters(BottomPayloadChoice);


%% ------------------------------------------------------------------------
% Motor / propeller derived parameters
% -------------------------------------------------------------------------

Config.Propulsion = ...
    propulsion_parameters( ...
        Config.Motor, ...
        Config.Propeller);

Config.MotorController = ...
    motor_controller_parameters( ...
        Config.Motor, ...
        Config.Propeller);


%% ------------------------------------------------------------------------
% Summary
% -------------------------------------------------------------------------

fprintf("\n");
fprintf("========================================\n");
fprintf(" Selected Configuration\n");
fprintf("========================================\n");

fprintf(" Drone          : %s\n", ...
    Config.Drone.Name);

fprintf(" Simulation     : %s\n", ...
    Config.Plant.Type);

fprintf(" Simulink model : %s\n", ...
    Config.Plant.Model);

fprintf(" Motor          : %s\n", ...
    Config.Motor.Name);

fprintf(" Propeller      : %s\n", ...
    Config.Propeller.Name);

fprintf(" Top payload    : %s\n", ...
    Config.TopPayload.Name);

fprintf(" Bottom payload : %s\n", ...
    Config.BottomPayload.Name);


fprintf("\n Propulsion:\n");

fprintf("   kT           : %.6e  (%s)\n", ...
    Config.Propulsion.kT, ...
    Config.Propulsion.kT_Source);

fprintf("   kQ           : %.6e  (%s)\n", ...
    Config.Propulsion.kQ, ...
    Config.Propulsion.kQ_Source);

fprintf("   Command RPM  : %.0f\n", ...
    Config.Propulsion.CommandMaxRPM);

fprintf("   Command source: %s\n", ...
    Config.Propulsion.CommandScaleSource);


fprintf("\n Motor controller:\n");

fprintf("   Type         : %s\n", ...
    Config.MotorController.Type);

fprintf("   Kp           : %.6f\n", ...
    Config.MotorController.Kp);

fprintf("   Ki           : %.6f\n", ...
    Config.MotorController.Ki);

fprintf("   Kd           : %.6f\n", ...
    Config.MotorController.Kd);

fprintf("   Source       : %s\n", ...
    Config.MotorController.Source);

fprintf("========================================\n\n");

end