function initialize_px4_tuning_session(project)
%INITIALIZE_PX4_TUNING_SESSION Build the base workspace a flight needs, without Main.mlx.
%   initialize_px4_tuning_session(project) selects the default drone configuration (the same choices
%   as Main.mlx), builds Config, Vehicle, SimParams, SimscapeParams, CommParams, IC, PX4Params and the
%   Default PX4Tune profile, and assigns them in the base workspace. Used by the runners and scripts
%   that fly without Main.mlx; IC is level at the origin. The runners call it only when the base
%   workspace is not initialised yet, so a workspace set up by Main.mlx is kept.
addpath(project);project_paths(project);
MotorOptions=motor_parameters();PropellerOptions=propeller_parameters();
DroneOptions=drone_parameters();PlantOptions=plant_parameters();
TopPayloadOptions=top_payload_parameters();BottomPayloadOptions=bottom_payload_parameters();
Config=load_configuration(DroneOptions(1),PlantOptions(2),MotorOptions(1), ...
    PropellerOptions(2),TopPayloadOptions(2),BottomPayloadOptions(1));
Vehicle=assemble_vehicle(Config);TopPayloadVariant=Config.TopPayload.VariantValue;
SimParams=build_simulation_parameters(Config,Vehicle);
SimscapeParams=build_simscape_parameters(Config,Vehicle,SimParams);
CommParams=communication_parameters();
IC=struct('Position',[0;0;0],'Velocity',[0;0;0],'EulerAngles',[0;0;0], ...
    'AngularVelocity',[0;0;0],'MotorOmega',[0;0;0;0]);
Aero_On=1;PX4Tune=px4_tuning_parameters("Default");
PX4Params=build_px4_parameters(Config,Vehicle,SimParams);
names={'Config','Vehicle','TopPayloadVariant','SimParams','SimscapeParams','CommParams','IC','Aero_On','PX4Tune','PX4Params'};
for k=1:numel(names),assignin('base',names{k},eval(names{k}));end
end
