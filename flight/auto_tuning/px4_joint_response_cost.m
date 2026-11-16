function F=px4_joint_response_cost(time,reference,measured,windows,amplitude,motorVariation)
%PX4_JOINT_RESPONSE_COST Joint step-response score of one loop ("joint_response_v1").
%   F = px4_joint_response_cost(time,reference,measured,windows,amplitude,motorVariation) scores the
%   step windows: Score = Tracking + 0.5*LateError + 0.5*OvershootRMS + 0.75*Oscillation
%   + 0.1*MotorVariation (response terms normalised by the step amplitude), and flags Usable when the
%   quality screen passes (tracking <= 0.5, late error <= 0.2, overshoot <= 0.15, oscillation <= 0.15,
%   motor variation <= 0.5). F also holds every term separately.
% Response-only objective: no stored gains, reference controller or hand tune.
time=double(time(:));reference=double(reference(:));measured=double(measured(:));
assert(amplitude>0&&isequal(size(time),size(reference),size(measured)));
tracking=[];tails=[];excess=[];ring=[];
for k=1:size(windows,1)
    on=windows(k,1);off=windows(k,2);holdTime=off-on;
    mask=time>=on+.05&time<off-.05&isfinite(measured)&isfinite(reference);
    assert(nnz(mask)>=3,'Incomplete response-cost window.');
    t=time(mask);r=reference(mask)/amplitude;y=measured(mask)/amplitude;
    error=r-y;tracking=[tracking;error]; %#ok<AGROW>
    tails=[tails;error(t>=on+.75*holdTime)]; %#ok<AGROW>
    excess=[excess;max(0,sign(r).*y-abs(r))]; %#ok<AGROW>
    width=max(3,2*floor(.15/max(median(diff(t)),eps)/2)+1);
    residual=y-movmean(y,width);
    inner=t>=on+max(.2,.2*holdTime)&t<=off-.15;
    ring=[ring;residual(inner)]; %#ok<AGROW>
end
assert(~isempty(tracking)&&~isempty(tails));
F=struct('Tracking',rms(tracking),'LateError',rms(tails), ...
    'OvershootRMS',rms(excess),'Oscillation',0,'MotorVariation',motorVariation/.05);
if ~isempty(ring),F.Oscillation=rms(ring);end
% Use the already-logged final zero-reference hold as an additional I check.
post=time>=max(windows(:,2))+5&time>=max(time)-5&isfinite(measured)&isfinite(reference);
if nnz(post)>=3,F.LateError=max(F.LateError,rms((reference(post)-measured(post))/amplitude));end
F.Score=F.Tracking+.5*F.LateError+.5*F.OvershootRMS+.75*F.Oscillation+.1*F.MotorVariation;
F.Usable=F.Tracking<=.5&&F.LateError<=.2&&F.OvershootRMS<=.15&& ...
    F.Oscillation<=.15&&F.MotorVariation<=.5;
assert(isfinite(F.Score),'Nonfinite joint objective.');
end
