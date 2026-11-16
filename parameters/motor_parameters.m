function Motor = motor_parameters(MotorChoice)
%MOTOR_PARAMETERS Catalogue of brushless motors.
%   Options = motor_parameters() lists the available motors.
%   Motor = motor_parameters(MotorChoice) returns the selected motor: ID, Name, KV, winding resistance
%   R, back-EMF and torque constants (ke, kt), current/power limits, test data, mass and size, time
%   constant, the Simscape motor block parameters and the command range. Used by load_configuration.
%
%   The motors are standard commercial size classes ("2212": 22 mm stator diameter, 12 mm stator
%   height) with values typical of their datasheets; they are not a specific product.

%% Available motors
MotorOptions = [ ...
    "2212_920KV"
    "2212_1400KV"
];

%% Return list if requested
if nargin == 0
    Motor = MotorOptions;
    return
end

%% Selected motor
switch MotorChoice

    case "2212_920KV"

        % Typical 10-inch-propeller motor on a 3S battery (the classic 2212 920KV + 10x4.5 pairing).
        Motor = generic2212("2212_920KV","2212 920KV",920,0.117,15,170,11.1,0.4,56e-3);

    case "2212_1400KV"

        % Typical 8-inch-propeller motor on a 3S battery.
        Motor = generic2212("2212_1400KV","2212 1400KV",1400,0.065,20,250,11.1,0.6,52e-3);

    otherwise

        error("Unknown motor: %s", MotorChoice)

end

end

function Motor = generic2212(id,name,KV,R,maxCurrent,maxPower,testVoltage,idleCurrent,mass)
% Common structure of the 2212-class motors.
Motor = struct();

%% Identification
Motor.ID   = string(id);
Motor.Name = string(name);

%% Electrical properties
Motor.KV = KV;                                 % [rpm/V]
Motor.R = R;                                   % [Ohm]
Motor.ke = 60/(2*pi*Motor.KV);                 % [V*s/rad]
Motor.kt = Motor.ke;                           % [N*m/A]
Motor.Max_Current = maxCurrent;                % [A]
Motor.Max_Power = maxPower;                    % [W]
Motor.Test_Voltage = testVoltage;              % [V]
Motor.Idle_Current_10V = idleCurrent;          % [A]

%% Physical properties (2212 class: 22 mm stator diameter, 12 mm stator height)
Motor.Mass = mass;                             % [kg]
Motor.Radius = 28e-3/2;                        % [m]
Motor.Height = 30e-3;                          % [m]
Motor.StatorDiameter = 22e-3;                  % [m]
Motor.StatorHeight = 12e-3;                    % [m]

%% Simplified mathematical motor model
Motor.Time_Constant = 0.1;                     % [s]

%% Simscape Motor & Drive parameters
% Maximum torque from the ideal SI torque constant (tau_max = kt * I_max). Efficiency is
% idealised at 100 %; the torque-control time constant is a model assumption.
Motor.Simscape.MaxTorque = Motor.kt * Motor.Max_Current;          % [N*m]
Motor.Simscape.MaxPower = Motor.Max_Power;                        % [W]
Motor.Simscape.TorqueControlTimeConstant = 0.0003;                % [s], model assumption
Motor.Simscape.Efficiency = 100;                                  % [%], idealised
Motor.Simscape.EfficiencyMeasurementSpeed = 0.85*KV*testVoltage;  % [rpm], reference operating point
Motor.Simscape.EfficiencyMeasurementTorque = 0.5 * Motor.Simscape.MaxTorque;
Motor.Simscape.Source = "Typical datasheet values + idealised Simscape motor assumptions";
Motor.Simscape.Validated = false;

%% Command limits
Motor.Command_Min = 0;
Motor.Command_Max = 1;
end
