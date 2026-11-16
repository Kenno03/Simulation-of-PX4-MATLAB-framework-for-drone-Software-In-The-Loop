function stages=px4_tuning_selection(number)
%PX4_TUNING_SELECTION Map a single-loop menu number (px4_tuning_menu, 0-16) to tuning stage keys.
%   stages = px4_tuning_selection(number) returns the stage keys for run_px4_tuning, for example
%   2 -> ["rollrate","pitchrate","yawrate"]; 0 returns no stage (cancel). 15 (a separate acceleration
%   loop) is not exposed and raises an error. The mapping is also checked by the offline tests.
switch number
    case 1,stages="all";
    case 2,stages=["rollrate","pitchrate","yawrate"];
    case 3,stages="rollrate";
    case 4,stages="pitchrate";
    case 5,stages="yawrate";
    case 6,stages=["rollattitude","pitchattitude","yawattitude"];
    case 7,stages="rollattitude";
    case 8,stages="pitchattitude";
    case 9,stages=["horizontalvelocity","verticalvelocity"];
    case 10,stages="horizontalvelocity";
    case 11,stages="verticalvelocity";
    case 12,stages=["horizontalposition","verticalposition"];
    case 13,stages="horizontalposition";
    case 14,stages="verticalposition";
    case 16,stages="yawattitude";
    case 0,stages=strings(1,0);
    case 15,error('px4:UnsupportedAcceleration','No separate acceleration-loop stage is exposed. The _ACC gains are velocity PID gains; choose Velocity (9).');
    otherwise,error('px4:MenuSelection','Choose a number from 0 to 16.');
end
end
