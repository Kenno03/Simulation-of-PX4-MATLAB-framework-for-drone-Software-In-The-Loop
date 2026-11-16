function MotorController = motor_controller_parameters(Motor, Propeller)
%MOTOR_CONTROLLER_PARAMETERS Internal Simscape rotor-speed controller (PI) for a motor/propeller pair.
%   MotorController = motor_controller_parameters(Motor, Propeller) returns PairID, Type, the PI
%   gains Kp, Ki, Kd and their source. These are NOT the PX4 attitude/rate/position gains: they only
%   close the simulated motor-speed loop (a near-ideal speed response). All catalogue pairs use the
%   same initial tuning; it is not validated against measured motor data.

pairID = Motor.ID + "__" + Propeller.ID;

MotorController = struct();
MotorController.PairID = pairID;
MotorController.Type = "PI";
MotorController.Kp = 0.015;
MotorController.Ki = 0.030;
MotorController.Kd = 0;
MotorController.Source = "Initial Simscape rotor-speed tuning (all motor/propeller pairs)";
MotorController.Validated = false;

end
