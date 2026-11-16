function [axisNow,segmentTime,amplitude]=px4_tuning_segment(testTime,mode,testAxis,baseAmplitude)
%PX4_TUNING_SEGMENT Map the test clock to the axis currently being excited.
%   [axisNow,segmentTime,amplitude] = px4_tuning_segment(testTime,mode,testAxis,baseAmplitude)
% Single-axis tests (testAxis 1..3) are unchanged: that axis, the same clock and amplitude.
% Multi-axis tests encode the axes as testAxis = 8 + bitmask (1 = x/roll, 2 = y/pitch,
% 4 = z/yaw). The selected axes are excited one after another, each with its own complete
% ten-pulse schedule and recovery (one segment = px4_tuning_step_windows duration).
% Outside any segment axisNow = 0 and segmentTime = -1, so no pulse is commanded.
%#codegen
axisNow=uint8(0);segmentTime=-1;amplitude=single(baseAmplitude);
t=double(testTime);code=double(testAxis);
if code<8
    axisNow=uint8(code);segmentTime=t;return;
end
[~,~,duration]=px4_tuning_step_windows(mode);
mask=code-8;order=zeros(1,3);n=0;
for k=1:3
    if mod(floor(mask/2^(k-1)),2)==1
        n=n+1;order(n)=k;
    end
end
if n==0||t<0,return;end
segment=floor(t/duration)+1;
if segment>n,return;end
axisNow=uint8(order(segment));
segmentTime=t-(segment-1)*duration;
amplitude=single(double(baseAmplitude)*px4_tuning_axis_scale(mode,order(segment)));
end
