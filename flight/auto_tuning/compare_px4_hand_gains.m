function Comparison=compare_px4_hand_gains(Session)
%COMPARE_PX4_HAND_GAINS Table of a tuning session's start and best gains next to the hand-tuned gains.
%   Comparison = compare_px4_hand_gains(Session) lists, per PX4 gain, the start, best and hand-tuned
%   (real drone) values and their differences, plus the effective rate gains K*P, K*I, K*D.
% Drone-specific reference comparison. It never changes or applies gains.
% Raw rate gains are not unique: PX4 multiplies P, I and D by the axis K.
hand=px4_tuning_parameters("HandTuned_RealDrone");
gain=px4_controller_gain_names()';
starting=arrayfun(@(name)double(Session.StartGains.(name)),gain);
best=arrayfun(@(name)double(Session.BestGains.(name)),gain);
reference=arrayfun(@(name)double(hand.(name)),gain);
difference=best-reference;
percent=nan(size(best));nonzero=reference~=0;
percent(nonzero)=100*difference(nonzero)./abs(reference(nonzero));
attempted=ismember(gain,Session.Results.Parameter);
validTrial=Session.Results.Status=="VALID";
validlyTested=ismember(gain,Session.Results.Parameter(validTrial));
if isfield(Session,'JointHistory')&&~isempty(Session.JointHistory)
    attempted=false(size(gain));validlyTested=attempted;
    for k=1:height(Session.JointHistory)
        changed=split(Session.JointHistory.ChangedGains(k),',');
        attempted=attempted|ismember(gain,changed);
        if Session.JointHistory.Valid(k),validlyTested=validlyTested|ismember(gain,changed);end
    end
end
inherited=["MC_ROLLRATE_K","MC_ROLLRATE_P","MC_ROLLRATE_I","MC_ROLLRATE_D", ...
    "MC_PITCHRATE_K","MC_PITCHRATE_I","MC_YAWRATE_D","MC_YAW_P"];
source=repmat("Explicit hand-tuned value",size(gain));
source(ismember(gain,inherited))="Inherited PX4 default";
Comparison.Raw=table(gain,starting,best,reference,difference,percent,attempted,validlyTested,source, ...
    'VariableNames',{'Gain','Starting','SessionBest','HandReference','Difference', ...
    'DifferencePercent','Attempted','ValidlyTested','HandValueSource'});
axisName=strings(9,1);term=axisName;
initialEffective=zeros(9,1);bestEffective=initialEffective;handEffective=initialEffective;
row=0;
for axis=["ROLL","PITCH","YAW"]
    for component=["P","I","D"]
        row=row+1;axisName(row)=axis;term(row)=component;
        name="MC_"+axis+"RATE_"+component;scale="MC_"+axis+"RATE_K";
        initialEffective(row)=double(Session.StartGains.(name))*double(Session.StartGains.(scale));
        bestEffective(row)=double(Session.BestGains.(name))*double(Session.BestGains.(scale));
        handEffective(row)=double(hand.(name))*double(hand.(scale));
    end
end
effectiveDifference=bestEffective-handEffective;
effectivePercent=nan(9,1);nonzero=handEffective~=0;
effectivePercent(nonzero)=100*effectiveDifference(nonzero)./abs(handEffective(nonzero));
Comparison.EffectiveRate=table(axisName,term,initialEffective,bestEffective,handEffective, ...
    effectiveDifference,effectivePercent,'VariableNames', ...
    {'Axis','Term','StartingKTimesGain','BestKTimesGain','HandKTimesGain','Difference','DifferencePercent'});
Comparison.SessionStatus=string(Session.Status);
fixed=endsWith(gain,"RATE_K");
Comparison.Raw.FixedNormalization=fixed;
Comparison.AllIndependentGainsValidlyTested=all(validlyTested(~fixed));
Comparison.Note="Gain-value comparison only. Similar values do not demonstrate similar flight performance; no fit to hand reference is used.";
Comparison.GeneratedAt=datetime('now');
end
