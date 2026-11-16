function M=score_px4_tuning_candidate(plant,controller,S,cfg)
%SCORE_PX4_TUNING_CANDIDATE Score one axis of a tuning flight from its step tests.
%   M = score_px4_tuning_candidate(plantData,controllerData,Stage,cfg) checks the flight (complete
%   steps, altitude, body-rate, recovery and motor-saturation limits; it errors if one fails) and
%   returns:
%     RMSE score  RMSE/A + 0.30*max|e|/A + 0.25*overshoot (+ altitude penalty), A = step amplitude.
%                 This is M.Score, or M.LegacyTrackingScore when the joint objective is active.
%     joint score with cfg.ObjectiveVersion "joint_response_v1": M.Score = px4_joint_response_cost,
%                 all terms in M.JointFeatures.
%   plus RMSE, Overshoot, MinAltitude, MaxRate, RecoveryPeak and StepCount.
M=struct('Score',inf,'RMSE',NaN,'Overshoot',NaN, ...
    'MinAltitude',NaN,'MaxRate',NaN,'RecoveryPeak',NaN,'StepCount',0);
D=px4_tuning_trial_data(plant,controller,S,cfg);
tr=D.Time;r=D.Reference;tm=D.MeasuredTime;actual=D.Measured;
nominal=D.Nominal;tp=D.PlantTime;p=[zeros(numel(tp),2),D.Altitude];
tw=D.RateTime;omega=D.BodyRates;offset=D.ClockOffset;

[ts,relativeTime]=asMatrix(readSignal(controller,"TUNING_TEST_TIME"));
released=isfinite(relativeTime) & relativeTime>=0;
assert(any(released),'Takeoff/settling gate did not release the tuning test.');
testStart=median(ts(released)+offset-relativeTime(released));
assert(isfinite(testStart),'Missing schedule clock alignment.');
% Multi-axis flights excite axes one after another; each axis is scored on its own segment.
if isfield(S,'SegmentOffset'),testStart=testStart+double(S.SegmentOffset);end
[tmode,modeValues]=asMatrix(readSignal(controller,"TUNING_MODE"));
activeMode=interp1(tmode+offset,modeValues,tr,'previous',NaN);
relative=tr-testStart;
[usable,windows,signs,recovery,malformed]=px4_tuning_completed_steps(D,S,testStart);
assert(~any(malformed),'Reference does not match stage amplitude/sign in a scheduled pulse.');
M.StepCount=nnz(usable);
assert(M.StepCount>=5,'Fewer than 5 complete usable steps with recovery were captured (%d of target 10).',M.StepCount);
last=find(usable,1,'last');
testDuration=windows(last,2)+recovery;
threshold=0.4*double(S.Amplitude);
expected=false(size(tr));
% Commands and measurements above are converted from Down to altitude for Z.
for w=find(usable)'
    interior=relative>=windows(w,1)+0.05 & relative<windows(w,2)-0.05;
    assert(nnz(interior)>2,'Missing commanded samples in scheduled pulse %d.',w);
    assert(all(activeMode(interior)==double(S.Mode)), ...
        'Wrong controller mode during scheduled excitation.');
    assert(all(abs((r(interior)-nominal)-signs(w)*double(S.Amplitude)) ...
        <max(1e-5,0.05*double(S.Amplitude))), ...
        'Reference does not match stage amplitude/sign in scheduled pulse %d.',w);
    expected=expected|interior;
end
pulseSamples=find(expected & isfinite(r) & abs(r-nominal)>threshold);
assert(numel(pulseSamples)>8,'No candidate excitation appeared in controller SDI.');
% Build contiguous pulse windows from the logged reference. This accommodates
% different chart schedules for rate, attitude, velocity and position tests.
breaks=[1;find(diff(tr(pulseSamples))>0.25)+1;numel(pulseSamples)+1];
pulseMask=false(size(tr));
for segment=1:numel(breaks)-1
    run=pulseSamples(breaks(segment):breaks(segment+1)-1);
    if numel(run)<3,continue;end
    inner=run(2:end-1); pulseMask(inner)=true;
end
good=pulseMask & isfinite(r);
sampleTime=tr(good); reference=r(good)-nominal;
assert(numel(sampleTime)>8 && max(reference)>threshold && min(reference)<-threshold, ...
    'Positive and negative excitation pulse segments were not both captured.');
good=isfinite(tm)&isfinite(actual); [tm,ix]=unique(tm(good)); actual=actual(good); actual=actual(ix);
assert(min(tm)<=testStart && max(tm)>=testStart+testDuration, ...
    'Plant log does not cover the complete scheduled test and recovery.');
y=interp1(tm,actual,sampleTime,'linear',NaN)-nominal;
err=reference-y; valid=isfinite(err)&isfinite(reference)&isfinite(y);
err=err(valid); reference=reference(valid); y=y(valid);
assert(numel(err)>8,'Insufficient synchronized response samples.');
if double(S.Mode)==2,err=atan2(sin(err),cos(err));end
M.RMSE=sqrt(mean(err.^2));
M.Overshoot=max(0,max(sign(reference).*y-abs(reference)))/double(S.Amplitude);
[~,~,targetDuration]=px4_tuning_step_windows(S.Mode);
safetyEnd=min(testStart+targetDuration,min([max(tp),max(tw),max(tr)]));
active=tp>=testStart & tp<=safetyEnd;
M.MinAltitude=min(p(active,3));
altAtStart=interp1(tp,p(:,3),testStart,'linear',NaN);
assert(isfinite(altAtStart) && altAtStart>cfg.MinTestAltitude, ...
    'Vehicle was not above the minimum test altitude before excitation.');
rateMask=tw>=testStart & tw<=safetyEnd;
M.MaxRate=max(abs(omega(rateMask,:)),[],'all');
lastPulse=testStart+windows(last,2);
M.RecoveryPeak=max(abs(omega(tw>=lastPulse & tw<=lastPulse+1,:)),[],'all');
assert(M.MinAltitude>=cfg.MinAltitude,'Altitude safety limit crossed.');
assert(M.MaxRate<=cfg.MaxBodyRate,'Body-rate safety limit crossed.');
assert(M.RecoveryPeak<=cfg.MaxRecoveryRate,'Recovery-rate safety limit crossed.');
maxMotor=0;
for motorIdx=1:4
    motorSignal=readSignal(plant,"Motor "+motorIdx);
    [motorTime,motorCommand]=asMatrix(motorSignal);
    onWindow=motorTime>=testStart & motorTime<=safetyEnd;
    assert(any(onWindow),'Missing motor-command samples in test window.');
    maxMotor=max(maxMotor,max(motorCommand(onWindow,:),[],'all'));
end
assert(maxMotor<0.99,'Motor command entered the saturation rejection region.');
M.Score=M.RMSE/double(S.Amplitude)+ ...
    0.30*max(abs(err))/double(S.Amplitude)+0.25*M.Overshoot+ ...
    0.02*max(0,cfg.MinTestAltitude-M.MinAltitude);
if isfield(cfg,'ObjectiveVersion')&&cfg.ObjectiveVersion=="joint_response_v1"
    variation=[];
    for motorIdx=1:4
        [motorTime,u]=asMatrix(readSignal(plant,"Motor "+motorIdx));
        grid=(testStart:.01:safetyEnd)';motor=interp1(motorTime,u,grid,'linear');
        variation=[variation;diff(motor)]; %#ok<AGROW>
    end
    fullMeasured=interp1(tm,actual,tr,'linear',NaN)-nominal;
    M.LegacyTrackingScore=M.Score;
    M.JointFeatures=px4_joint_response_cost(relative,r-nominal,fullMeasured, ...
        windows(usable,:),double(S.Amplitude),rms(variation,'all'));
    M.Score=M.JointFeatures.Score;
end
assert(isfinite(M.Score),'Candidate score is not finite.');
end

function v=readSignal(d,query)
names=string(d.getElementNames);
idx=find(names==query);
assert(numel(idx)<=1,'Ambiguous SDI signal: %s.',query);
assert(~isempty(idx),'SDI signal not found: %s. Available: %s',query,strjoin(names,', '));
e=d.getElement(idx); v=e.Values;
end

function [t,y]=asMatrix(s)
assert(isa(s,'timeseries'),'Expected timeseries SDI signal, got %s.',class(s));
t=double(s.Time(:)); a=double(s.Data);
if s.IsTimeFirst, y=reshape(a,numel(t),[]); else, y=reshape(a,[],numel(t))'; end
end

