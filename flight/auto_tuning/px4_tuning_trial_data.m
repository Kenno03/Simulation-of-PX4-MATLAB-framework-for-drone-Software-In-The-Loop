function D=px4_tuning_trial_data(plant,controller,S,cfg)
%PX4_TUNING_TRIAL_DATA Align the commanded reference with the plant truth for one tuning flight.
%   D = px4_tuning_trial_data(plantData,controllerData,Stage,cfg) reads the plant's Drone State and the
%   PX4_Receive logs, maps controller time onto plant time (controller_clock_offset) and returns the
%   reference, the measured response of the stage's axis, altitude, body rates and the clock offset.
%   Used by score_px4_tuning_candidate and write_px4_trial_report.
% Shared command/physical-truth alignment for scoring and per-trial reports.
state=readSignal(plant,"Drone State");
assert(isstruct(state),'Plant Drone State bus was not exported.');
[tp,pned]=asMatrix(state.S_NED); [te,etaRaw]=asMatrix(state.Eta_NED);
[tw,omega]=bodyRates(state);
p=pned; p(:,3)=-p(:,3); % altitude positive upward for safety checks
eta=etaRaw(:,[3 2 1]);
offset=controller_clock_offset(controller);

truthTime=[];truthMeasured=[];measurementSource="Plant physical truth";
switch double(S.Mode)
    case 1
        ref=readSignal(controller,"TUNING_CMD_RATE_"+upper(axisName(S.Axis)));
        [tr,r]=asMatrix(ref); actual=omega(:,double(S.Axis)); tm=tw;
        nominal=0;
    case 2
        qref=readSignal(controller,"TUNING_CMD_ATTITUDE"); [tr,q]=asMatrix(qref);
        rr=quatEuler(q); r=rr(:,double(S.Axis));
        tm=te; actual=eta(:,double(S.Axis)); nominal=0;
        if double(S.Axis)==3
            % EKF heading is the yaw of the same quaternion used by PX4 attitude
            % control. Keep plant yaw separately: a fixed alignment offset is
            % estimator/frame information, not an error that yaw P can remove.
            truthTime=tm;truthMeasured=actual;
            [tm,actual]=asMatrix(readSignal(controller,"<heading>"));tm=tm+offset;
            measurementSource="PX4 EKF heading feedback";
        end
    case {3,4}
        nm="TUNING_CMD_POSITION"; if double(S.Mode)==3,nm="TUNING_CMD_VELOCITY";end
        ref=readSignal(controller,nm); [tr,r3]=asMatrix(ref);
        r=r3(:,double(S.Axis));
        if double(S.Mode)==3
            v=state.Sdot_NED; [tm,vel]=asMatrix(v); actual=vel(:,double(S.Axis));
            if double(S.Axis)==3, actual=-actual; r=-r; end
            nominal=0;
        else
            actual=p(:,double(S.Axis)); tm=tp;
            if double(S.Axis)==3, r=-r; end
            nominal=double(cfg.HomeNED(double(S.Axis)));
            if double(S.Axis)==3, nominal=-nominal; end
        end
    otherwise
        error('Unsupported tuning mode %d.',double(S.Mode));
end
tr=tr+offset;
D=struct('Time',tr,'Reference',r,'MeasuredTime',tm,'Measured',actual, ...
    'Nominal',nominal,'PlantTime',tp,'Altitude',p(:,3), ...
    'RateTime',tw,'BodyRates',omega,'EulerTime',te,'Euler',eta, ...
    'ClockOffset',offset,'MeasurementSource',measurementSource, ...
    'TruthMeasuredTime',truthTime,'TruthMeasured',truthMeasured);
end

function [t,omega]=bodyRates(state)
if isfield(state,'Omega_frd')
    [t,omega]=asMatrix(state.Omega_frd);
elseif isfield(state,'Omega_NED')
    % Legacy bus alias: source was the body-FRD signal, verified in model.
    [t,omega]=asMatrix(state.Omega_NED);
else
    assert(isfield(state,'Omega_nav_ned') && isfield(state,'R_ned_to_frd'), ...
        'Body rates require Omega_frd or the logged navigation rate and rotation.');
    [t,nav]=asMatrix(state.Omega_nav_ned);
    [tr,R]=asMatrix(state.R_ned_to_frd);
    assert(size(nav,2)==3 && size(R,2)==9,'Unexpected logged rate/rotation dimensions.');
    R=interp1(tr,R,t,'linear',NaN);
    assert(all(isfinite(R),'all'),'Missing recorded rotation coverage for body rates.');
    % Each row of R is the column-major flattening of the recorded 3x3 matrix.
    omega=[sum(R(:,[1 4 7]).*nav,2), ...
        sum(R(:,[2 5 8]).*nav,2),sum(R(:,[3 6 9]).*nav,2)];
end
end

function name=axisName(axis)
names=["roll","pitch","yaw"];
name=names(double(axis));
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

function e=quatEuler(q)
assert(size(q,2)==4,'Expected scalar-first [w x y z] quaternion.');
w=q(:,1);x=q(:,2);y=q(:,3);z=q(:,4);
e=[atan2(2*(w.*x+y.*z),1-2*(x.^2+y.^2)), ...
    asin(min(max(2*(w.*y-z.*x),-1),1)), ...
    atan2(2*(w.*z+x.*y),1-2*(y.^2+z.^2))];
end

