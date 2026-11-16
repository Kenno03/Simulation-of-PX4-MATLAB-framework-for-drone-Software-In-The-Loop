function [usable,windows,signs,recovery,malformed]=px4_tuning_completed_steps(D,S,testStart)
%PX4_TUNING_COMPLETED_STEPS Which of the ten scheduled step pulses were completely recorded.
%   [usable,windows,signs,recovery,malformed] = px4_tuning_completed_steps(D,S,testStart) checks each
%   pulse of the stage's schedule (px4_tuning_step_windows) against the trial data D
%   (px4_tuning_trial_data): command, response and recovery must all be logged. MALFORMED marks
%   pulses whose recorded reference does not match the schedule.
% Identify completed scheduled pulses with recorded command, response/recovery.
[windows,signs,duration]=px4_tuning_step_windows(S.Mode);
recovery=duration-windows(end,2);
if ismember(double(S.Mode),[3 4])&&double(S.Axis)==3,signs=-signs;end
usable=false(10,1);malformed=false(10,1);
coverage=min([max(D.Time),max(D.MeasuredTime),max(D.PlantTime),max(D.RateTime)]);
for k=1:10
    if coverage<testStart+windows(k,2)+recovery,continue;end
    inside=D.Time>=testStart+windows(k,1)+.05 & D.Time<testStart+windows(k,2)-.05;
    if nnz(inside)<3,continue;end
    command=D.Reference(inside)-D.Nominal;
    if all(isfinite(command)&abs(command)<.05*double(S.Amplitude)),continue;end
    matches=all(isfinite(command)&abs(command-signs(k)*double(S.Amplitude)) ...
        <max(1e-5,.05*double(S.Amplitude)));
    if ~matches,malformed(k)=true;continue;end
    measured=interp1(D.MeasuredTime,D.Measured,D.Time(inside),'linear',NaN);
    usable(k)=all(isfinite(measured));
end
end
