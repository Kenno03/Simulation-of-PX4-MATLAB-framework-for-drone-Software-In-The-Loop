function [A,out,note]=px4_rmse_axis(action,A,varargin)
%PX4_RMSE_AXIS Per-axis gain search that minimises the RMSE step-response score.
%   Same interface as px4_guide_axis, so run_px4_guide_tuning can fly all axes of a loop group in one
%   multi-axis flight (Method="rmse"):
%     A = px4_rmse_axis("init",[],Group,memberIndex,tune)
%     A = px4_rmse_axis("baseline",A,metricsArray)      repeat flights -> start score and noise margin
%     [A,gains,note] = px4_rmse_axis("propose",A)        next gains to fly ("hold" when done)
%     [A,gains,note] = px4_rmse_axis("observe",A,metrics) report the flown result
%     [~,gains] = px4_rmse_axis("probe",A,factor)        current gains with the loop gain x factor
%     [A,gains] = px4_rmse_axis("backoff",A,factor)      scale the loop gain (robustness probe failed)
%
%   Score (lower is better), per axis on its step tests (score_px4_tuning_candidate):
%       J = RMSE/A + 0.30*max|e|/A + 0.25*overshoot          (A = step amplitude)
%   Search: coordinate (pattern) search in log-gain space over the axis's search knobs. Each knob is
%   tried up (x Step) and, if that does not help, down (/ Step). A change is accepted only if J drops
%   by more than the repeat-noise margin (2 standard deviations of the baseline repeats) and the
%   response is valid with no sustained oscillation. When a full cycle over all knobs brings no
%   accepted change, the step is refined once (1.3 -> 1.14); the axis is done after a further cycle
%   without improvement or after MaxProposals flights.
%   Knobs: roll/pitch rate K (overall), D and I; yaw rate K and I; attitude P; velocity P, I, D;
%   position P. Rate P is scaled through K, so the P:I:D ratios change only through D and I.
%   PX4 resolution: every proposed, probed or backed-off gain is rounded to the decimals PX4 accepts
%   for it (px4_controller_gain_limits). A step that rounds back to the current value, or to gains
%   this axis has already flown, is skipped without a flight.
out=struct();note="";
switch string(action)
    case "init",A=initAxis(varargin{:});out=A.Gains;
    case "baseline",A=baseline(A,varargin{1});out=A.Gains;
    case "propose",[A,out,note]=propose(A);
    case "observe",[A,note]=observe(A,varargin{1});out=A.Gains;
    case "backoff",A=backoff(A,varargin{1});out=A.Gains;
    case "probe",out=scaleLoop(A,A.Gains,varargin{1});   % gains for the robustness probe; A unchanged
    otherwise,error('px4_rmse_axis:action','Unknown action %s.',action);
end
end

%% ------------------------------------------------------------------------
function A=initAxis(G,j,tune)
m=G.Members(j);f=m.Gains;names=string(struct2cell(f))';
g=struct();for n=names,g.(n)=double(tune.(n));end
switch G.Key
    case "rates"
        if m.Axis~=3,knobs=[f.K f.D f.I];else,knobs=[f.K f.I];end
    case "velocity",knobs=[f.P f.I f.D];
    otherwise,knobs=f.P;   % attitude, position
end
knobs=knobs(arrayfun(@(n)g.(n)~=0,knobs));   % a zero gain (e.g. yaw D) stays zero
A=struct('Group',G.Key,'Key',m.Key,'Name',m.Name,'GainFields',f,'Gains',g,'Knobs',knobs, ...
    'Step',1.3,'MinStep',1.14,'Refined',false,'KnobIndex',1,'Direction',+1,'CycleImproved',false, ...
    'BestScore',inf,'Noise',0.01,'OscLimit',0.15,'Done',false,'Pending',struct('Kind',"none"), ...
    'Proposals',0,'MaxProposals',14,'History',table(),'Log',strings(0,1));
end

function A=baseline(A,M)
v=M([M.Valid]&isfinite([M.RMSEScore]));
if isempty(v),A.Done=true;A.Log(end+1)="Baseline: no valid repeat; gains kept.";return;end
s=[v.RMSEScore];A.BestScore=mean(s);
if numel(s)>=2,A.Noise=max(0.005*A.BestScore,2*std(s));else,A.Noise=0.02*A.BestScore;end
A.Log(end+1)=sprintf('Baseline %d valid repeats: RMSE score %.4f, noise margin %.4f.',numel(s),A.BestScore,A.Noise);
A=addHistory(A,"baseline",A.Gains,A.BestScore,true);
end

%% ------------------------------------------------------------------------
function [A,out,note]=propose(A)
out=A.Gains;
if A.Done||A.Proposals>=A.MaxProposals||isempty(A.Knobs)
    if ~A.Done,A.Done=true;A.Log(end+1)=sprintf('Search stopped after %d proposals; best score %.4f.',A.Proposals,A.BestScore);end
    A.Pending=struct('Kind',"hold");note="done";return;
end
n=A.Knobs(A.KnobIndex);
if A.Direction>0,f=A.Step;else,f=1/A.Step;end
value=snap(n,A.Gains.(n)*f);out.(n)=value;
% Skip without flying: the step rounds back to the current value (at a bound, or smaller than the
% PX4 resolution), or these gains were already flown for this axis.
if abs(value-A.Gains.(n))<1e-12*max(1,abs(value))||alreadyFlown(A,out)
    A=advance(A,false);[A,out,note]=propose(A);return;
end
A.Proposals=A.Proposals+1;
A.Pending=struct('Kind',"step",'Knob',n,'Gains',out);
note=sprintf('rmse %s x%.3g',n,value/A.Gains.(n));
end

function [A,note]=observe(A,met)
note="";if A.Pending.Kind~="step",return;end
g=A.Pending.Gains;ok=met.Valid&&~met.Unstable&&isfinite(met.RMSEScore)&&met.Oscillation<=A.OscLimit;
better=ok&&met.RMSEScore<A.BestScore-A.Noise;
A=addHistory(A,A.Pending.Knob,g,met.RMSEScore,better);
if better
    A.Gains=g;A.BestScore=met.RMSEScore;A.CycleImproved=true;
    note=sprintf('%s accepted: score %.4f',A.Pending.Knob,met.RMSEScore);
    A.Log(end+1)=note;A=advance(A,true);
else
    if ~ok,why="invalid/oscillating";else,why=sprintf('score %.4f not better than %.4f',met.RMSEScore,A.BestScore);end
    note=sprintf('%s rejected (%s)',A.Pending.Knob,why);A=advance(A,false);
end
A.Pending=struct('Kind',"none");
end

function A=advance(A,improved)
% Keep going in an improving direction; otherwise try the other direction, then the next knob.
if improved,return;end
if A.Direction>0,A.Direction=-1;return;end
A.Direction=+1;A.KnobIndex=A.KnobIndex+1;
if A.KnobIndex>numel(A.Knobs)
    A.KnobIndex=1;
    if ~A.CycleImproved
        if ~A.Refined,A.Refined=true;A.Step=A.MinStep;A.Log(end+1)=sprintf('Cycle without improvement: step refined to x%.3g.',A.Step);
        else,A.Done=true;A.Log(end+1)=sprintf('Converged: best score %.4f.',A.BestScore);end
    end
    A.CycleImproved=false;
end
end

function A=backoff(A,factor)
A.Gains=scaleLoop(A,A.Gains,factor);
A.Log(end+1)=sprintf('Robustness probe failed: loop gain x%.3g.',factor);
end

function g=scaleLoop(A,g,factor)
% Overall loop gain: rate loops scale only K (K multiplies P, I and D); other loops scale every
% searched gain. Clipped to the PX4 limits.
for n=A.Knobs
    if A.Group=="rates"&&~endsWith(n,"_K"),continue,end
    g.(n)=snap(n,g.(n)*factor);
end
end

function v=snap(n,x)
% Clip to the PX4 bounds and round to the gain's PX4 resolution (px4_controller_gain_limits).
[lo,hi,res]=limits(n);v=min(max(round(x,round(-log10(res))),lo),hi);
end

function tf=alreadyFlown(A,g)
% True when this axis has already flown gains g (all search knobs equal within half a resolution step).
tf=false;
for k=1:height(A.History)
    h=A.History.Gains{k};same=true;
    for n=A.Knobs,[~,~,res]=limits(n);same=same&&abs(h.(n)-g.(n))<res/2;end
    if same,tf=true;return;end
end
end

function A=addHistory(A,knob,g,score,accepted)
row=table(numel(A.Log),string(knob),{g},score,accepted,'VariableNames',{'Step','Knob','Gains','RMSEScore','Accepted'});
if isempty(A.History),A.History=row;else,A.History=[A.History;row];end
end

function [lo,hi,res]=limits(n)
[lo,hi,res]=px4_controller_gain_limits(n);
if endsWith(n,"RATE_K"),lo=0.01;hi=5;end
end
