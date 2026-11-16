function scale=px4_tuning_axis_scale(mode,axis)
%PX4_TUNING_AXIS_SCALE Amplitude of an axis relative to its group's base amplitude.
%   scale = px4_tuning_axis_scale(mode,axis). Multi-axis velocity/position flights use the horizontal
%   amplitude (0.8) as base; the vertical axis keeps its established single-stage amplitude 0.6
%   (= 0.75 x 0.8). All other axes: 1. Runs in PX4_Receive (code generation).
%#codegen
scale=1;
if (double(mode)==3||double(mode)==4)&&double(axis)==3
    scale=0.75;
end
end
