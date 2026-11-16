function [mode,testTime] = px4_tuning_schedule(t,stage,testStart,z,vz,plan,minAltitude,maxClimbRate)
%PX4_TUNING_SCHEDULE Release the step test once the vehicle is airborne and settled (runs in PX4_Receive).
%   [mode,testTime] = px4_tuning_schedule(t,stage,testStart,z,vz,plan,minAltitude,maxClimbRate)
%   keeps position control (mode 4) during takeoff; once t >= testStart and the vehicle has stayed
%   above minAltitude with |vz| <= maxClimbRate for 4 s, it switches to the stage's mode and starts
%   the test clock testTime (-1 before the release).
% Position-controlled takeoff and measured settling precede loop excitation.
%#codegen
persistent stableSince actualStart lastTime
if isempty(lastTime)
    stableSince=-1.0;actualStart=-1.0;lastTime=-1.0;
end
if t<lastTime,stableSince=-1.0;actualStart=-1.0;end
lastTime=double(t);
ready=isfinite(z) && isfinite(vz) && ...
    -double(z)>double(minAltitude) && abs(double(vz))<=double(maxClimbRate);
if ~ready
    stableSince=-1.0;
elseif stableSince<0
    stableSince=double(t);
end
if actualStart<0 && t>=testStart && stableSince>=0 && ...
        t-stableSince>=plan.SettleTime
    actualStart=double(t);
end
if actualStart<0
    mode=4.0;
    testTime=min(double(t)-double(testStart),-1e-6);
else
    mode=double(stage);
    testTime=double(t)-actualStart;
end
end
