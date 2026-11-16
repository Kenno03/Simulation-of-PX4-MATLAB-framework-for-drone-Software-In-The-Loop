function [A,out,note]=px4_guide_axis(action,A,varargin)
%PX4_GUIDE_AXIS Per-axis decision logic that follows the PX4 tuning guide (PDF p. 72-137).
% Pure MATLAB (no simulation): the runner asks every axis of a loop group for its next gains,
% flies them all in one multi-axis flight, then reports each axis's measured response back.
%   A = px4_guide_axis("init",[],Group,memberIndex,tune)
%   [A,gains,note] = px4_guide_axis("propose",A)
%   [A,gains,note] = px4_guide_axis("observe",A,metrics)
%   A = px4_guide_axis("baseline",A,metricsArray)   % repeat flights -> noise margins
% metrics fields: Valid, Unstable, Overshoot [%], Tracking, LateError, Oscillation, Score.
out=struct();note="";
switch string(action)
    case "init",A=initAxis(varargin{:});out=A.Gains;
    case "baseline",A=baseline(A,varargin{1});out=A.Gains;
    case "propose",[A,out,note]=propose(A);
    case "observe",[A,note]=observe(A,varargin{1});out=A.Gains;
    otherwise,error('Unknown action %s.',action);
end
end

%% ------------------------------------------------------------------------
function A=initAxis(G,j,tune)
m=G.Members(j);names=string(struct2cell(m.Gains))';
g=struct();for n=names,g.(n)=double(tune.(n));end
A=struct('Group',G.Key,'Key',m.Key,'Name',m.Name,'Mode',double(G.Mode),'Axis',double(m.Axis), ...
    'GainFields',m.Gains,'Gains',g,'Phases',procedure(G.Key,m),'PhaseIndex',1,'Done',false, ...
    'Last',[],'LastValid',false,'Pending',struct('Kind',"none"),'Noise',defaultNoise(), ...
    'MeasureFailures',0,'Log',strings(0,1));
end

function phases=procedure(group,m)
f=m.Gains;phases=struct([]);
switch group
    case "rates"
        if m.Axis~=3
            % Clear D-term/high-gain oscillation first (guide p. 96: reduce P and D in 10% steps),
            % then P:D balance via D (p. 83-90), then the overall multiplier K (p. 91-100).
            phases=[phase("osc_fix",[f.P f.D],0.9,0.9,5), ...
                    phase("pd_balance",f.D,1.2,0.8,4,'Lim',[1 6]), ...
                    phase("ramp_until_osc",f.K,1.2,0.8,6,'Backoff',0.75)];
        else
            % Yaw: no D (p. 100); K like roll/pitch, then I for setpoint tracking (p. 103).
            phases=[phase("ramp_until_osc",f.K,1.2,0.8,6,'Backoff',0.75), ...
                    phase("yaw_i",f.I,1.5,1,3,'Lim',8,'Target',0.10)];
        end
    case "attitude"
        % Angle P until the response peak matches the target without overshoot (p. 108-110).
        phases=phase("ramp_until_overshoot",f.P,1.2,0.8,6,'Lim',3,'Backoff',0.75);
    case "velocity"
        % Often overdamped on defaults: reduce D first (p. 127); raise P until overshoot or the
        % 25 Hz P-oscillation (back off >=30%, p. 129-130); then scale P, I, D together (p. 118/128).
        phases=[phase("trial_step",f.D,0.5,1,1), ...
                phase("ramp_until_overshoot",f.P,1.2,0.8,5,'Lim',5,'Backoff',0.7), ...
                phase("ramp_until_overshoot",[f.P f.I f.D],1.2,0.8,4,'Lim',5,'Backoff',0.7)];
    case "position"
        % Raise P until position hold is accurate, but no further (p. 136-137).
        phases=phase("ramp_until_accurate",f.P,1.2,0.8,5,'Lim',5,'Target',0.05);
end
end

function P=phase(rule,knob,up,down,maxSteps,varargin)
P=struct('Rule',string(rule),'Knob',string(knob),'Up',up,'Down',down,'MaxSteps',maxSteps, ...
    'Lim',NaN,'Target',NaN,'Backoff',1,'Started',false,'Direction',0,'Steps',0,'M',1, ...
    'Base',struct(),'Start',struct(),'Hist',struct('M',{},'Metrics',{}));
for k=1:2:numel(varargin),P.(varargin{k})=varargin{k+1};end
end

function n=defaultNoise()
n=struct('Overshoot',1,'Tracking',0.005,'LateError',0.005,'Oscillation',0.03,'Score',0.01,'Repeats',0);
end

%% ------------------------------------------------------------------------
function A=baseline(A,M)
% Repeat flights at unchanged gains: mean = starting measurement, spread = decision margins.
v=M([M.Valid]);
if isempty(v),A.Last=M(end);A.LastValid=false;A.Log(end+1)="Baseline: no valid repeat.";return;end
f=["Overshoot","Tracking","LateError","Oscillation","Score"];
last=v(1);for k=f,last.(k)=mean([v.(k)],'omitnan');end
A.Last=last;A.LastValid=true;
sd=@(x)std(x(isfinite(x)));noise=defaultNoise();
if numel(v)>=2
    noise.Overshoot=max(1,2*sd([v.Overshoot]));noise.Tracking=max(0.005,2*sd([v.Tracking]));
    noise.LateError=max(0.005,2*sd([v.LateError]));noise.Oscillation=max(0.03,3*sd([v.Oscillation]));
    noise.Score=max(0.01,2*sd([v.Score]));
end
noise.Repeats=numel(v);A.Noise=noise;
A.Log(end+1)=sprintf('Baseline %d valid repeats: overshoot %.2f%% (margin %.2f), tracking %.4f, late %.4f, osc %.4f.', ...
    numel(v),last.Overshoot,noise.Overshoot,last.Tracking,last.LateError,last.Oscillation);
end

%% ------------------------------------------------------------------------
function [A,out,note]=propose(A)
out=A.Gains;
if A.Done,A.Pending=struct('Kind',"hold");note="done";return;end
if ~A.LastValid
    A.Pending=struct('Kind',"measure");note="measure current gains";return;
end
for guard=1:20
    if A.PhaseIndex>numel(A.Phases),A.Done=true;A.Pending=struct('Kind',"hold");note="procedure complete";return;end
    P=A.Phases(A.PhaseIndex);
    if ~P.Started
        P.Started=true;P.M=1;P.Steps=0;P.Base=knobValues(A,P);P.Start=A.Last;
        P.Hist=struct('M',1,'Metrics',A.Last);
        [P,done,finalM,why]=startRule(P,A.Last,A.Noise,A);
        A.Phases(A.PhaseIndex)=P;
        if done,A=finish(A,finalM,why);if ~A.LastValid,A.Pending=struct('Kind',"measure");note="measure after "+P.Rule;return;end,continue;end
    end
    if P.Direction>0,m=P.M*P.Up;else,m=P.M*P.Down;end
    [g,clipped]=gainsAt(A,P,m);
    if isequaln(g,A.Gains)
        A=finish(A,bestM(P),"bound reached");
        if ~A.LastValid,A.Pending=struct('Kind',"measure");note="measure after bound";return;end
        continue;
    end
    A.Pending=struct('Kind',"step",'M',m,'Clipped',clipped);out=g;
    note=sprintf('%s step %d: %s x%.3g',P.Rule,P.Steps+1,strjoin(P.Knob,'+'),m);return;
end
error('Guide procedure did not converge to a proposal for %s.',A.Key);
end

function [A,note]=observe(A,met)
note="";kind=A.Pending.Kind;
switch kind
    case {"hold","none"}
        return;
    case "measure"
        A.Last=met;A.LastValid=met.Valid;
        if ~met.Valid
            A.MeasureFailures=A.MeasureFailures+1;
            A.Log(end+1)="Measurement at current gains not valid (unstable="+string(met.Unstable)+").";
            if A.MeasureFailures>=2,A.Done=true;A.Log(end+1)="Stopped: repeated invalid measurements at current gains.";end
        end
        note="measured";return;
end
P=A.Phases(A.PhaseIndex);m=A.Pending.M;P.Steps=P.Steps+1;
P.Hist(end+1)=struct('M',m,'Metrics',met);
[P,done,finalM,why]=stepRule(P,met,A.Noise,m,A.Pending.Clipped);
A.Phases(A.PhaseIndex)=P;
if done
    A.Phases(A.PhaseIndex).M=m;
    A=finish(A,finalM,why);
    if finalM==m&&met.Valid,A.Last=met;A.LastValid=true;end
else
    A.Phases(A.PhaseIndex).M=m;[A.Gains,~]=gainsAt(A,P,m);A.Last=met;A.LastValid=met.Valid;
end
note=why;
end

%% ------------------------------------------------------------------------
function [P,done,finalM,why]=startRule(P,x,noise,A)
done=false;finalM=1;why="";oscLimit=0.15;
switch P.Rule
    case "osc_fix"
        if x.Oscillation<=oscLimit,done=true;why="no oscillation";else,P.Direction=-1;end
    case "pd_balance"
        % Overshoot cannot go below zero, so the noise margin applies to the overshoot side only.
        if x.Overshoot>=P.Lim(1)&&x.Overshoot<=P.Lim(2)+noise.Overshoot,done=true;why="P:D balanced";
        elseif x.Overshoot>P.Lim(2),P.Direction=+1;else,P.Direction=-1;end
    case "ramp_until_osc"
        if x.Oscillation>oscLimit,P.Direction=-1;else,P.Direction=+1;end
    case "ramp_until_overshoot"
        if x.Overshoot>P.Lim+noise.Overshoot,P.Direction=-1;else,P.Direction=+1;end
    case "ramp_until_accurate"
        if x.Overshoot>P.Lim+noise.Overshoot,P.Direction=-1;
        elseif x.LateError<=P.Target,done=true;why="hold already accurate";else,P.Direction=+1;end
    case "yaw_i"
        if x.LateError<=P.Target,done=true;why="yaw tracks setpoint";else,P.Direction=+1;end
    case "trial_step"
        vals=struct2cell(knobValues(A,P));
        if x.Overshoot<1&&all(cellfun(@(v)v>0,vals)),P.Direction=+1;P.Up=P.Up;else,done=true;why="not overdamped";end
end
end

function [P,done,finalM,why]=stepRule(P,x,noise,m,clipped)
done=false;finalM=m;why="continue";oscLimit=0.15;
prevM=P.Hist(end-1).M;startOsc=P.Start.Oscillation;
onset=x.Unstable||(x.Valid&&x.Oscillation>max(oscLimit,startOsc+noise.Oscillation));
atLimit=P.Steps>=P.MaxSteps||clipped;
switch P.Rule
    case "osc_fix"
        if x.Valid&&x.Oscillation<=oscLimit,done=true;why="oscillation cleared";
        elseif atLimit,done=true;finalM=argBest(P,'Oscillation');why="osc_fix limit";end
    case "pd_balance"
        lo=P.Lim(1);hi=P.Lim(2);mid=mean(P.Lim);
        if ~x.Valid,done=true;finalM=prevM;why="unstable; previous D kept";
        elseif x.Overshoot>=lo&&x.Overshoot<=hi,done=true;why="P:D balanced";
        elseif (P.Direction>0&&x.Overshoot<lo)||(P.Direction<0&&x.Overshoot>hi)
            done=true;prev=P.Hist(end-1).Metrics.Overshoot;
            if abs(prev-mid)<abs(x.Overshoot-mid),finalM=prevM;end;why="balance bracketed";
        elseif atLimit,done=true;finalM=argBestBy(P,@(h)abs(h.Overshoot-mid));why="pd limit";end
    case "ramp_until_osc"
        if P.Direction>0
            if onset,done=true;finalM=m*P.Backoff;why=sprintf('oscillation onset; back off to x%.2g',P.Backoff);
            elseif noGain(P,noise),done=true;finalM=argBest(P,'Tracking');why="diminishing returns";
            elseif atLimit,done=true;finalM=argBest(P,'Tracking');why="K limit";end
        else
            if x.Valid&&x.Oscillation<=oscLimit,done=true;why="oscillation removed";
            elseif atLimit,done=true;finalM=argBest(P,'Oscillation');why="K reduce limit";end
        end
    case "ramp_until_overshoot"
        if P.Direction>0
            if onset,done=true;finalM=min(prevM,m*P.Backoff);why="oscillation onset; backed off";
            elseif x.Overshoot>P.Lim+noise.Overshoot,done=true;finalM=prevM;why="overshoot; previous kept";
            elseif noGain(P,noise),done=true;finalM=argBest(P,'Tracking');why="diminishing returns";
            elseif atLimit,done=true;finalM=argBest(P,'Tracking');why="ramp limit";end
        else
            if x.Valid&&x.Overshoot<=P.Lim,done=true;why="overshoot removed";
            elseif atLimit,done=true;finalM=argBest(P,'Overshoot');why="reduce limit";end
        end
    case "ramp_until_accurate"
        if P.Direction>0
            if onset||x.Overshoot>P.Lim+noise.Overshoot,done=true;finalM=prevM;why="overshoot/oscillation; previous kept";
            elseif x.LateError<=P.Target,done=true;why="hold accurate";
            elseif x.LateError>bestOf(P,'LateError',true)-noise.LateError,done=true;finalM=argBest(P,'LateError');why="no further improvement";
            elseif atLimit,done=true;finalM=argBest(P,'LateError');why="ramp limit";end
        else
            if x.Valid&&x.Overshoot<=P.Lim,done=true;why="overshoot removed";
            elseif atLimit,done=true;finalM=argBest(P,'Overshoot');why="reduce limit";end
        end
    case "yaw_i"
        if onset||x.Overshoot>P.Lim+noise.Overshoot,done=true;finalM=prevM;why="I overshoot; previous kept";
        elseif x.LateError<=P.Target,done=true;why="yaw tracks setpoint";
        elseif x.LateError>bestOf(P,'LateError',true)-noise.LateError,done=true;finalM=argBest(P,'LateError');why="no further improvement";
        elseif atLimit,done=true;finalM=argBest(P,'LateError');why="I limit";end
    case "trial_step"
        done=true;
        if x.Valid&&x.Score<P.Start.Score-noise.Score,why="lower D kept";else,finalM=1;why="lower D reverted";end
end
end

%% ------------------------------------------------------------------------
function tf=noGain(P,noise)
% Two consecutive steps without tracking improvement beyond the noise margin.
tf=false;h=P.Hist;if numel(h)<3,return;end
t=arrayfun(@(e)e.Metrics.Tracking,h);best=min(t(1:end-2));
tf=t(end-1)>best-noise.Tracking&&t(end)>best-noise.Tracking;
end
function v=bestOf(P,field,excludeLast)
h=P.Hist;if excludeLast,h=h(1:end-1);end
x=arrayfun(@(e)value(e.Metrics,field),h);v=min(x);
end
function m=argBest(P,field)
x=arrayfun(@(e)value(e.Metrics,field),P.Hist);[~,i]=min(x);m=P.Hist(i).M;
end
function m=argBestBy(P,fn)
x=arrayfun(@(e)penalty(e.Metrics,fn),P.Hist);[~,i]=min(x);m=P.Hist(i).M;
end
function v=value(x,field)
if ~x.Valid||~isfinite(x.(field)),v=inf;else,v=x.(field);end
end
function v=penalty(x,fn)
if ~x.Valid,v=inf;else,v=fn(x);end
end
function m=bestM(P)
m=P.M;
end

function A=finish(A,finalM,why)
P=A.Phases(A.PhaseIndex);[g,~]=gainsAt(A,P,finalM);
changed=~isequaln(g,A.Gains);A.Gains=g;
if changed,A.LastValid=false;end
A.Log(end+1)=sprintf('%s (%s): %s -> x%.3g of phase start.',P.Rule,strjoin(P.Knob,'+'),why,finalM);
A.PhaseIndex=A.PhaseIndex+1;
if A.PhaseIndex>numel(A.Phases),A.Done=true;end
end

function v=knobValues(A,P)
v=struct();for n=P.Knob,v.(n)=A.Gains.(n);end
end

function [g,clipped]=gainsAt(A,P,m)
g=A.Gains;clipped=false;
for n=P.Knob
    target=P.Base.(n)*m;
    [lo,hi,res]=px4_controller_gain_limits(n);if endsWith(n,"RATE_K"),lo=0.01;hi=5;end
    value=min(max(target,lo),hi);
    clipped=clipped||abs(value-target)>1e-12*max(1,abs(target));   % at a PX4 bound (not rounding)
    value=min(max(round(value,round(-log10(res))),lo),hi);          % PX4 resolution
    if P.Base.(n)==0,value=0;end
    g.(n)=value;
end
end
