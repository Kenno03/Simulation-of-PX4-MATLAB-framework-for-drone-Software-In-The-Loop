function [windows,signs,duration]=px4_tuning_step_windows(mode)
%PX4_TUNING_STEP_WINDOWS Schedule of the ten step pulses of a tuning test.
%   [windows,signs,duration] = px4_tuning_step_windows(mode) returns the start/end time of each pulse
%   (10x2, seconds from the test start), their signs (+,-,+,...) and the total test duration including
%   recovery. Rates (mode 1): 1.5 s pulses every 3 s; attitude/velocity/position: 4 s every 8 s.
% Ten commanded departures from neutral, five positive and five negative.
%#codegen
% Return-to-neutral edges are not counted as additional test steps.
windows=zeros(10,2);signs=ones(1,10);
if mode==1,lead=1;spacing=3;holdTime=1.5;recovery=3;
else,lead=2;spacing=8;holdTime=4;recovery=4;end
for k=1:10
    windows(k,1)=lead+(k-1)*spacing;
    windows(k,2)=windows(k,1)+holdTime;
    if mod(k,2)==0,signs(k)=-1;end
end
duration=windows(10,2)+recovery;
end
