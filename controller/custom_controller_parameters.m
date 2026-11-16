function [CustomLQR, ValidatedFor] = custom_controller_parameters(design)
%CUSTOM_CONTROLLER_PARAMETERS Gains and limits of the custom flight controller (CUSTOM_CONTROLLER = 1).
%   CustomLQR = custom_controller_parameters()       LQI: 12-state LQR with integral action (default)
%   CustomLQR = custom_controller_parameters("lqr")  plain 12-state LQR (no integral action)
%   [CustomLQR, ValidatedFor] = ... also returns the setup the gains were designed for (total mass
%   [kg], inertia [kg*m^2], maximum total thrust [N]); check_gain_validity warns on another setup.
%
%   Design (linearize_drone on the fixed plant, 2026-11-11): discrete LQR at Ts = 0.010 s (the
%   PX4_Receive rate), Bryson Q with xs = [0.5 m, 1 m/s, 10 deg, 30 deg/s], R = 1000*I.
%   LQI: the state is augmented with the integrals of the N, E, D and yaw errors, weighted with
%   zs = [0.5 0.5 0.5] m*s and 0.5 rad*s. Linear checks on the full plant with motor dynamics:
%     LQR: disk phase margin at the motors 39 / 38 / 27 deg with 0 / 1 / 2 extra samples of delay;
%          constant disturbances leave a permanent offset (hover bias +0.01: 32 cm).
%     LQI: 34 / 33 / 25 deg; the same disturbances are removed within about 5 s.
%   These gains were designed for a different vehicle than the unit drone of this repository
%   (check_gain_validity warns). Redesign for the current setup with linearize_drone and
%   design_custom_controller(result, WriteParameters=true), which rewrites this file.
%
%   State order (linearize_drone): [N E D | vN vE vD | yaw pitch roll | p q r]. Output: rotor-speed
%   command m per motor (Simscape input), motor order = PX4 geometry M1..M4. See lqr_hover_control.
if nargin < 1, design = "lqi"; end
CustomLQR.Ts = 0.01;
% Hover rotor-speed command: the linearisation point of the design.
CustomLQR.m0 = [0.52605948;0.52605948;0.52605948;0.52605948];
switch lower(string(design))
    case "lqi"
        CustomLQR.K = [ ...
    -0.0593782799 -0.0536851798 -0.0725787963 -0.0487692447 -0.0442098308 -0.0652978871 0.116663867 0.198429685 -0.181139686 -0.0290312303 0.0401072502 0.0831090489
    -0.0574183368 0.0593509593 -0.0657475928 -0.0471530829 0.0487767733 -0.0590765581 -0.121108448 0.191006998 0.198793038 0.0316085553 0.0391528935 -0.0858093168
    0.0575584161 -0.0589431485 -0.0748462955 0.0472748776 -0.048401032 -0.0673053349 -0.106932455 -0.192067764 -0.195806894 -0.0315270517 -0.0388352759 -0.0754050874
    0.0556957707 0.0647842596 -0.067829711 0.0457427779 0.0531093069 -0.060989935 0.110736109 -0.185120156 0.213947922 0.0344640437 -0.0382176972 0.0777969171
            ];
        % integral gains, columns: integrals of the N, E, D [m*s] and yaw [rad*s] errors
        CustomLQR.Ki = [ ...
    -0.0279601203 -0.0252459962 -0.0322266154 0.0319580207
    -0.0270427065 0.0279375017 -0.0292259392 -0.0330674768
    0.0271025886 -0.0277613569 -0.0332457631 -0.029171696
    0.0262292441 0.0305360509 -0.0301364159 0.0300774814
            ];
    case "lqr"
        CustomLQR.K = [ ...
    -0.0280725752 -0.0253660661 -0.0323261861 -0.0342608534 -0.03103499 -0.0453155742 0.0911439709 0.171300656 -0.156169824 -0.028042973 0.0391339719 0.0749448874
    -0.0271454458 0.0280694912 -0.0293014276 -0.0331215397 0.034284182 -0.0410038341 -0.0947914692 0.164849513 0.171722394 0.0306218212 0.0381712172 -0.0774594849
    0.0272122138 -0.0278506905 -0.0333334585 0.0332135684 -0.0339755179 -0.0467019621 -0.0838405661 -0.165814046 -0.168844594 -0.0304871851 -0.0378597057 -0.0681132015
    0.026329796 0.0306363066 -0.0302283797 0.0321319306 0.037323917 -0.0423341482 0.0868774455 -0.159768098 0.184821009 0.0334250768 -0.0372358232 0.0702513659
            ];
        CustomLQR.Ki = zeros(4,4);
    otherwise
        error('custom_controller_parameters:design','Unknown design "%s" (use "lqi" or "lqr").',design);
end
CustomLQR.MaxPosError = 1.0;     % [m] error limit per axis (linear region)
CustomLQR.MaxVelError = 2.0;     % [m/s]
CustomLQR.MaxYawError = 0.5;     % [rad] yaw error limit (linear region)
CustomLQR.MaxIntegralPos = 1.8;  % [m*s] integral limit per axis (at most 0.06 motor command)
CustomLQR.MaxIntegralYaw = 1.81;  % [rad*s] yaw integral limit (at most 0.06 motor command)
CustomLQR.MinCmd = 0.05;         % lowest rotor-speed command in flight
CustomLQR.IdleCmd = 0.2;         % armed on the ground before the takeoff reference starts
% Setup the design was made for: a different vehicle, not part of this repository (unknown = NaN).
ValidatedFor = struct('Mass',NaN,'Inertia',[NaN NaN NaN],'MaxThrust',NaN);
end
