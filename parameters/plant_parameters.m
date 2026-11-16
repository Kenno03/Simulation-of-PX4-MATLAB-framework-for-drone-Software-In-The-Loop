function Plant = plant_parameters(PlantChoice)
%PLANT_PARAMETERS Catalogue of plant implementations of the drone model.
%   Options = plant_parameters() lists the plant types ("Simulink", "Simscape").
%   Plant = plant_parameters(PlantChoice) returns Type, Model (model file) and VariantValue, which
%   selects the plant variant in Drone_simulation_PX4. Used by load_configuration.

%% Available plant types
PlantOptions = [
    "Simulink"
    "Simscape"
];

%% Return options if no input
if nargin == 0
    Plant = PlantOptions;
    return
end

%% Selected plant
switch PlantChoice

    case "Simulink"

        Plant = struct();

        Plant.Type         = "Simulink";
        Plant.Model        = "Drone_simulation_PX4";
        Plant.VariantValue = 1;

    case "Simscape"

        Plant = struct();

        Plant.Type         = "Simscape";
        Plant.Model        = "Simscape_Drone_Model";
        Plant.VariantValue = 2;

    otherwise
        error("Unknown plant model: %s", PlantChoice)

end

end