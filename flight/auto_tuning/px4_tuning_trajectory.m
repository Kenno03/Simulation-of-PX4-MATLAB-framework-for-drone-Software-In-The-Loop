function [position,velocity,acceleration,jerk,yaw,yawspeed] = ...
    px4_tuning_trajectory(testTime,mode,axis,amplitude,home,plan,testStart)
%PX4_TUNING_TRAJECTORY Trajectory setpoint during a tuning flight (runs in PX4_Receive).
%   [position,velocity,acceleration,jerk,yaw,yawspeed] = px4_tuning_trajectory(testTime,mode,axis, ...
%       amplitude,home,plan,testStart)
%   Before the test: the hover plan's takeoff to HOME. Velocity tests (mode 3): velocity pulses on the
%   active axis; position tests (mode 4): position pulses around HOME. Unused fields are NaN (PX4
%   ignores them). Rate and attitude tests are commanded by their own setpoint blocks.
% Background takeoff is distinct from velocity/position loop excitation.
%#codegen
% axis may be a single axis (1..3) or a multi-axis code (see px4_tuning_segment).
position=single(nan(1,3)); velocity=single(nan(1,3));
acceleration=single(nan(1,3)); jerk=single(nan(1,3));
yaw=single(0); yawspeed=single(0);
if testTime<0
    [position,velocity,acceleration,jerk,yaw,yawspeed] = ...
        flight_trajectory(double(testTime)+double(testStart),plan);
    return;
end
[axisNow,segmentTime,segmentAmplitude]=px4_tuning_segment(testTime,mode,axis,amplitude);
cmd=px4_tuning_step_command(segmentTime,mode,segmentAmplitude);
idx=int32(axisNow);
if mode==3
    velocity=single(zeros(1,3));
    if idx>=1 && idx<=3,velocity(idx)=cmd;end
elseif mode==4
    position=single(reshape(home,1,3));
    if idx>=1 && idx<=3,position(idx)=position(idx)+cmd;end
end
end
