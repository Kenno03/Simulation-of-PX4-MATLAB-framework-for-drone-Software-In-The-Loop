function [Aol,Bol,Col,Dol,result] = linearize_drone(options)
%LINEARIZE_DRONE Linearise the drone plant at hover (state-space model for controller design).
% Linearise the selected plant at known hover input and initial conditions.
% No optimisation or trimming is performed. One short snapshot simulation is
% used by default; SnapshotTime=0 linearises directly at the initial point.
% Single-file entry point. Examples:
%   [Aol,Bol,Col,Dol] = linearize_drone;      % existing Main selection
%   [Aol,Bol,Col,Dol,r] = linearize_drone(Plant="Simscape");
%   [Aol,Bol,Col,Dol] = linearize_drone(Plant="Simulink");
%   [~,~,~,~,r] = linearize_drone(BuildOnly=true); % harness only
% Run Main.mlx's hardware/plant initialisation first. Its Config, SimParams,
% IC and optional SimscapeParams are reused. No PX4 process is needed.
% HoverInput is an optional vector of normalised actuator commands; otherwise
% SimParams.HoverMotorCommand or the Main hover thrust/rotor geometry is used.
% Simscape uses Config.Plant.Model; Simulink uses the mathematical PlantBlock.
% Each has one actuator-vector input and two outputs; NativeStatePort selects
% DroneBus (default port 2). The other port is auxiliary data. The state bus
% exposes StateSignals below. Original source models are never saved.
% State vector: [S_NED; Sdot_NED; Eta_NED; Omega_nav_ned], in bus order.
% FlightStatesOnly=true returns a 12-state quasi-steady actuator approximation.
% The unreduced linearisation remains in result.FullModel. Use
% FlightStatesOnly=false to return that full model instead.
% Generated analysis models retain the selected parameters in model workspaces.
arguments
    options.ProjectRoot (1,1) string = ""
    options.Plant (1,1) string = ""
    options.BuildOnly (1,1) logical = false
    options.PlantBlock (1,1) string = "Drone_simulation_PX4/Drone Model"
    options.NativeStatePort (1,1) double {mustBeMember(options.NativeStatePort,[1 2])} = 2
    options.StateSignals (1,4) string = ["State.S_NED","State.Sdot_NED", ...
        "State.Eta_NED","State.Omega_nav_ned"]
    options.FlightStatesOnly (1,1) logical = true
    options.HoverInput double = []
    options.SnapshotTime (1,1) double {mustBeNonnegative} = 0.1
    options.HoverPositionNED (3,1) double = [0;0;-1]
    options.Yaw (1,1) double = 0
    options.LevelHover (1,1) logical = true
    options.Validate (1,1) logical = false
    options.ValidationTime (1,1) double {mustBePositive} = 0.3
    options.Perturbation (1,1) double {mustBePositive} = 0.001
    options.OutputFolder (1,1) string = ""
end
here=fileparts(mfilename('fullpath'));
root=char(options.ProjectRoot);
if isempty(root)
    % the project root is the folder of initialization.m (the parameter files are in parameters/)
    if isfile(fullfile(here,'initialization.m')), root=here;
    elseif isfile(fullfile(pwd,'initialization.m')), root=pwd;
    else, root=fileparts(which('initialization')); end
end
assert(isfile(fullfile(root,'initialization.m')),'Specify ProjectRoot for the vehicle project.');
oldPath=path;
pathGuard=onCleanup(@()path(oldPath));
addpath(root);project_paths(root);
names={'Config','Vehicle','SimParams','SimscapeParams','CommParams','IC','Aero_On', ...
    'TopPayloadVariant','LinHoverCommand'};
saved=cell(size(names)); existed=false(size(names));
for k=1:numel(names)
    existed(k)=evalin('base',sprintf('exist(''%s'',''var'')==1',names{k}));
    if existed(k), saved{k}=evalin('base',names{k}); end
end
baseGuard=onCleanup(@()restoreWorkspace(names,existed,saved));
assert(existed(1) && existed(3), ...
    'Run Main.mlx hardware/plant initialisation first (Config and SimParams are required).');
Config=saved{1};
if options.Plant~="", Config.Plant=plant_parameters(options.Plant); end
assignin('base','Config',Config);
if options.OutputFolder==""
    folder=fullfile(root,'linearization_results',char(datetime('now','Format','yyyyMMdd_HHmmss_SSS')));
else, folder=char(options.OutputFolder); end
assert(~isfolder(folder),'OutputFolder must be new to avoid overwriting an earlier result.');
mkdir(folder);
addpath(folder);
oldGen=Simulink.fileGenControl('getConfig');
genGuard=onCleanup(@()Simulink.fileGenControl('setConfig','config',oldGen));
Simulink.fileGenControl('set','CacheFolder',fullfile(folder,'cache'), ...
    'CodeGenFolder',fullfile(folder,'codegen'),'createDir',true);
evalc('prepareParameters(Config,options.HoverInput)');
if evalin('base','exist(''IC'',''var'')'), IC=evalin('base','IC');
else, IC=struct; end
IC.Position=[options.HoverPositionNED(2);options.HoverPositionNED(1);-options.HoverPositionNED(3)];
IC.Velocity=zeros(3,1); IC.AngularVelocity=zeros(3,1);
if ~isfield(IC,'EulerAngles'), IC.EulerAngles=zeros(3,1); end
params=evalin('base','SimParams');
IC.MotorOmega=evalin('base','LinHoverCommand').*params.CommandMaxOmega(:);
assignin('base','IC',IC);
hover=evalin('base','LinHoverCommand');
[~,tag]=fileparts(tempname);
tag=matlab.lang.makeValidName(tag);
tag=tag(max(1,end-11):end);
mdl=['linearize_model_' tag];
ref=['lin_plant_' tag];
analysis=['lin_physics_' tag];
modelGuard=onCleanup(@()closeAnalysisModels({mdl,ref,analysis}));
buildHarness(root,folder,mdl,ref,analysis,options.PlantBlock,options.StateSignals,options.HoverPositionNED,options.Yaw,options.LevelHover,options.NativeStatePort);
if options.BuildOnly
    Aol=[]; Bol=[]; Col=[]; Dol=[];
    result=struct('Folder',folder,'Harness',fullfile(folder,[mdl '.slx']));
    fprintf('Created referenced-plant harness: %s\n',result.Harness);
    return
end
assert(license('test','Simulink_Control_Design'),'Simulink Control Design is required.');
nu=numel(hover);
io=getlinio(mdl);
linopt=linearizeOptions('BlockReduction','on');
fprintf('Known hover commands: %s\n',mat2str(hover',6));
if options.SnapshotTime==0
    fprintf('Linearising the initial point at 1 m hover (no simulation/search).\n');
    [sys,op,info]=linearize(mdl,io,linopt);
else
    fprintf('Simulating %.3g s with fixed hover commands, then linearising once. No optimisation.\n',options.SnapshotTime);
    [sys,op,info]=linearize(mdl,io,options.SnapshotTime,linopt);
end
linop=op;
assert(all(isfinite([sys.A(:);sys.B(:);sys.C(:);sys.D(:)])), ...
    'Linearisation produced nonfinite matrix entries.');
sys.InputName=cellstr("motor_"+string((1:nu)'));
flightNames={'S_NED(1)','S_NED(2)','S_NED(3)', ...
    'Sdot_NED(1)','Sdot_NED(2)','Sdot_NED(3)', ...
    'Eta_NED(1)','Eta_NED(2)','Eta_NED(3)', ...
    'Omega_nav_ned(1)','Omega_nav_ned(2)','Omega_nav_ned(3)'};
sys.OutputName=flightNames;
sys.InputUnit=repmat({'1'},nu,1);
sys.OutputUnit=[repmat({'m'},3,1);repmat({'m/s'},3,1); ...
    repmat({'rad'},3,1);repmat({'rad/s'},3,1)];
fullModel=sys;
reduction=struct('Method','none');
if options.FlightStatesOnly
    [sys,reduction]=flightStateModel(sys);
end
[A,B,C,D]=ssdata(sys);
Aol=A; Bol=B; Col=C; Dol=D;
n=size(Aol,1);
Mc=ctrb(Aol,Bol);
Mo=obsv(Aol,Col);
rankControllability=NaN; rankObservability=NaN;
if all(isfinite(Mc(:))), rankControllability=rank(Mc); end
if all(isfinite(Mo(:))), rankObservability=rank(Mo); end
lambda=eig(Aol);
pbhControl=zeros(n,1); pbhObserve=zeros(n,1);
for k=1:n
    pbhControl(k)=rank([lambda(k)*eye(n)-Aol,Bol]);
    pbhObserve(k)=rank([lambda(k)*eye(n)-Aol;Col]);
end
isControllable=all(pbhControl==n);
isObservable=all(pbhObserve==n);
fprintf('\nControllability: rank(ctrb(Aol,Bol)) = %g / %d\n',rankControllability,n);
fprintf('PBH controllability cross-check: minimum rank = %d / %d\n',min(pbhControl),n);
if isControllable
    fprintf('The extracted system is controllable.\n');
else
    warning('linearize_drone:NotControllable', ...
        'The extracted system is NOT fully controllable (minimum PBH rank %d of %d states).',min(pbhControl),n);
end
fprintf('Observability: rank(obsv(Aol,Col)) = %g / %d\n',rankObservability,n);
fprintf('PBH observability cross-check: minimum rank = %d / %d\n',min(pbhObserve),n);
if isObservable
    fprintf('The extracted system is observable.\n');
else
    warning('linearize_drone:NotObservable', ...
        'The extracted system is NOT fully observable (minimum PBH rank %d of %d states).',min(pbhObserve),n);
end
if (isControllable && rankControllability~=n) || (isObservable && rankObservability~=n)
    warning('linearize_drone:RankConditioning', ...
        ['The raw ctrb/obsv rank tests lose numerical rank because their powers of Aol are poorly scaled. ' ...
        'Use the printed PBH cross-check and inspect the stored diagnostics; matrices have not been altered to force a full rank.']);
end
u0=hover;
y0=[];
result=struct('sys',sys,'Aol',Aol,'Bol',Bol,'Col',Col,'Dol',Dol, ...
    'Harness',fullfile(folder,[mdl '.slx']),'OperatingPoint',op, ...
    'LinearizationOperatingPoint',linop,'Info',info,'u0',u0, ...
    'FullModel',fullModel,'Reduction',reduction, ...
    'Config',Config,'InitialConditions',IC,'SnapshotTime',options.SnapshotTime, ...
    'StateNames',{sys.StateName},'Folder',folder);
result.ControllabilityRank=rankControllability;
result.ObservabilityRank=rankObservability;
result.IsControllable=isControllable;
result.IsObservable=isObservable;
result.PBHControllabilityRanks=pbhControl;
result.PBHObservabilityRanks=pbhObserve;
% A/B use Simulink's actual internal states, not an assumed twelve-state order.
result.ValidationPerformed=options.Validate;
result.ValidatedHoverLTI=[];
if options.Validate
    result.Validation=validateLocal(mdl,sys,op,y0,options,folder);
    result.ValidatedHoverLTI=result.Validation.Passed;
end
% The saved harness retains the prescribed IC and known motor commands.
save(fullfile(folder,'linearization.mat'),'result','sys','Aol','Bol','Col','Dol','u0','op');
writetable(table((1:size(A,1))',string(sys.StateName), ...
    'VariableNames',{'Index','InternalState'}),fullfile(folder,'state_order.csv'));
writematrix(A,fullfile(folder,'A.csv')); writematrix(B,fullfile(folder,'B.csv'));
writematrix(C,fullfile(folder,'C.csv')); writematrix(D,fullfile(folder,'D.csv'));
% Return the result without adding controller-design variables to base workspace.
fprintf('Saved %d-state, %d-input, %d-output model to %s\n',size(A,1),size(B,2),size(C,1),folder);
end

function [reduced,details]=flightStateModel(full)
% Retain the requested flight coordinates, not arbitrary rows of native A.
[A,B,C,D]=ssdata(full);
nx=size(A,1); nf=size(C,1);
assert(nf==12 && rank(C)==12,'The four selected signals must provide 12 independent flight variables.');
assert(norm(D,'fro')<1e-10,'Flight variables with direct input feedthrough require a different state mapping.');
[~,~,pivot]=qr(C,'vector');
flight=pivot(1:nf);
remaining=setdiff(1:nx,flight,'stable');
frozen=[];
for k=remaining
    if ~isempty(regexpi(full.StateName{k},'(propeller|rotor|shaft).*\.(q|angle|theta|phi)$','once'))
        frozen(end+1)=k; %#ok<AGROW>
    end
end
actuators=setdiff(remaining,frozen,'stable');
active=[flight,actuators];
na=numel(actuators);
T=[C(:,active);zeros(na,nf),eye(na)];
assert(rcond(T)>1e-12,'The selected flight-coordinate transformation is numerically singular.');
At=T*A(active,active)/T;
Bt=T*B(active,:);
Ar=At(1:nf,1:nf); Br=Bt(1:nf,:);
if na>0
    A12=At(1:nf,nf+1:end); A21=At(nf+1:end,1:nf);
    A22=At(nf+1:end,nf+1:end); B2=Bt(nf+1:end,:);
    if full.Ts==0
        assert(rcond(A22)>1e-12, ...
            'Internal dynamics cannot be residualised. Use FlightStatesOnly=false to inspect the full model.');
        Ar=Ar-A12*(A22\A21); Br=Br-A12*(A22\B2);
        stable=all(real(eig(A22))<0);
    else
        M=eye(na)-A22;
        assert(rcond(M)>1e-12,'Internal discrete dynamics cannot be residualised.');
        Ar=Ar+A12*(M\A21); Br=Br+A12*(M\B2);
        stable=all(abs(eig(A22))<1);
    end
    if ~stable
        warning('linearize_drone:UnstableInternalDynamics', ...
            'Eliminated actuator dynamics are not stable; the flight-only approximation needs review.');
    end
end
reduced=ss(Ar,Br,eye(nf),zeros(nf,size(B,2)),full.Ts);
reduced.StateName=full.OutputName; reduced.OutputName=full.OutputName;
reduced.InputName=full.InputName; reduced.InputUnit=full.InputUnit;
reduced.OutputUnit=full.OutputUnit;
details=struct('Method','flight-coordinate quasi-steady actuator residualisation', ...
    'OriginalStateCount',nx,'FlightCoordinateColumns',flight, ...
    'EliminatedActuatorStates',{full.StateName(actuators)}, ...
    'FrozenCyclicStates',{full.StateName(frozen)});
if nx>nf
    warning('linearize_drone:FlightStateApproximation', ...
        ['Returning the 12 selected flight variables. Internal actuator dynamics are treated as quasi-steady ' ...
        'and %d cyclic rotor-phase deviations are held fixed. The full %d-state model is retained in result.FullModel.'], ...
        numel(frozen),nx);
end
end

function validation=validateLocal(mdl,sys,op,y0,options,folder)
t=(0:0.001:options.ValidationTime)';
if t(end)<options.ValidationTime, t(end+1)=options.ValidationTime; end
trimDelta=op.Inputs(1).u(:)';
baseInput=repmat(trimDelta,numel(t),1);
base=simulateFromOp(mdl,op,t,baseInput);
% A collective perturbation plus each actuator works for any motor count/order.
nu=size(sys.B,2);
patterns=[ones(nu,1),eye(nu)];
labels=["collective","motor_"+string(1:nu)];
validation=struct('Time',t,'Baseline',base,'Cases',[]);
if isempty(y0), validation.ReplayInitialError=NaN;
else, validation.ReplayInitialError=max(abs(base(1,4:12)'-y0(4:12))); end
validation.NominalDrift=max(abs(base-base(1,:)),[],1);
validation.NominalHoverMaintained=all(validation.NominalDrift<1e-3);
if ~validation.NominalHoverMaintained
    warning('linearize_drone:NonstationaryOperatingPoint', ...
        ['The unperturbed nonlinear plant leaves the trimmed point over the validation horizon. ' ...
        'The matrices are a frozen local linearisation, not a validated hover LTI model.']);
end
for j=1:size(patterns,2)
    du=options.Perturbation*repmat(patterns(:,j)',numel(t),1);
    actual=simulateFromOp(mdl,op,t,baseInput+du)-base;
    predicted=lsim(sys,du,t);
    err=actual-predicted;
    rmsError=sqrt(mean(err.^2,1));
    peak=max(abs(actual),[],1);
    relative=rmsError./max(peak,1e-7);
    validation.Cases(j).Name=labels(j);
    validation.Cases(j).DeltaInput=du;
    validation.Cases(j).NonlinearDelta=actual;
    validation.Cases(j).LinearDelta=predicted;
    validation.Cases(j).RMSError=rmsError;
    validation.Cases(j).RelativeRMSError=relative;
    validation.Cases(j).Passed=all(rmsError<=1e-6+0.05*peak);
    f=figure('Visible','off','Position',[50 50 1250 850]);
    guard=onCleanup(@()close(f));
    tiledlayout(4,3,'TileSpacing','compact');
    for k=1:12
        nexttile; plot(t,actual(:,k),'-',t,predicted(:,k),'--','LineWidth',1.1);
        grid on; title(sys.OutputName{k},'Interpreter','none');
        xlabel('Time (s)'); ylabel(sys.OutputUnit{k});
    end
    legend('Nonlinear perturbation','Linear prediction');
    sgtitle(sprintf('%s: motor perturbation %.4g',labels(j),options.Perturbation));
    exportgraphics(f,fullfile(folder,char(labels(j)+".png")));
    clear guard
end
validation.Note='Perturbed nonlinear trajectory minus unperturbed trajectory, both starting at the same trimmed operating point.';
validation.Passed=all([validation.Cases.Passed]) && ...
    validation.NominalHoverMaintained;
if ~validation.Passed
    warning('linearize_drone:ValidationMismatch', ...
        'Some nonlinear comparisons exceed 5%% of peak response plus 1e-6. Inspect the exported plots before controller design.');
end
end

function y=simulateFromOp(mdl,op,t,input)
in=Simulink.SimulationInput(mdl);
in=in.setInitialState(getstatestruct(op));
start=op.Time;
in=in.setExternalInput([t+start input]);
in=in.setModelParameter('StopTime',num2str(start+t(end),17),'StartTime',num2str(start,17), ...
    'SaveOutput','on','SaveFormat','Dataset', ...
    'OutputSaveName','yout','ReturnWorkspaceOutputs','on');
out=sim(in);
ts=out.yout.getElement(1).Values;
data=ts.Data;
if size(data,1)==numel(ts.Time)
    data=reshape(data,numel(ts.Time),[]);
else
    data=reshape(data,[],numel(ts.Time)).';
end
[time,idx]=unique(ts.Time-start,'last');
y=interp1(time,data(idx,:),t,'linear','extrap');
end

function buildHarness(root,here,mdl,ref,analysis,plantBlock,stateSignals,hoverPosition,yaw,levelHover,statePort)
plantType=string(evalin('base','Config.Plant.Type'));
new_system(mdl);
storeContext(mdl);
set_param(mdl,'PostLoadFcn',sprintf( ...
    'addpath(fileparts(get_param(''%s'',''FileName'')));',mdl));
set_param(mdl,'SolverType','Variable-step','Solver','ode23t', ...
    'MaxStep','0.002','RelTol','1e-6','AbsTol','1e-8', ...
    'StopTime','1','ReturnWorkspaceOutputs','on','SaveOutput','on', ...
    'OutputSaveName','yout','SaveFormat','Dataset','SignalLogging','on', ...
'Location',[100 100 1300 760]);
add_block('simulink/Sources/In1',[mdl '/Motor perturbation'], ...
    'PortDimensions',num2str(numel(evalin('base','LinHoverCommand'))),'Position',[50 190 80 210]);
add_block('simulink/Sources/Constant',[mdl '/Hover command'], ...
    'Value','LinHoverCommand','Position',[45 100 170 140]);
add_block('simulink/Math Operations/Add',[mdl '/Motor command'], ...
    'Inputs','++','Position',[215 160 245 205]);
add_block('simulink/Ports & Subsystems/Subsystem',[mdl '/Selected plant'], ...
    'Position',[310 135 550 285]);
Simulink.SubSystem.deleteContents([mdl '/Selected plant']);
add_block('simulink/Sources/In1',[mdl '/Selected plant/Motor command'], ...
    'Position',[35 100 65 120]);
switch plantType
    case "Simscape"
        % Only the configured Simscape model is referenced in this branch.
        model=evalin('base','Config.Plant.Model');
        add_block('built-in/ModelReference',[mdl '/Selected plant/Plant variant'], ...
            'ModelName',char(model),'SimulationMode','Normal','Position',[140 60 380 200]);
    case "Simulink"
        % Only the mathematical plant is copied; no Simscape model is loaded.
        source=extractBefore(plantBlock,'/');
        if ~bdIsLoaded(source), load_system(which(source+'.slx')); end
        choice=plantBlock+"/Matchematical Drone Model";
        if getSimulinkBlockHandle(choice)>0, plantBlock=choice; end
        add_block(plantBlock,[mdl '/Selected plant/Plant variant'],'Position',[140 60 380 200]);
        set_param([mdl '/Selected plant/Plant variant'],'VariantControl','');
    otherwise
        error('Select Simscape or Simulink using Config.Plant.Type.');
end
add_block('simulink/Sinks/Out1',[mdl '/Selected plant/DroneBus'], ...
    'Port','1','Position',[450 90 480 110]);
add_block('simulink/Sinks/Out1',[mdl '/Selected plant/DroneData'], ...
    'Port','2','Position',[450 160 480 180]);
add_line([mdl '/Selected plant'],'Motor command/1','Plant variant/1');
add_line([mdl '/Selected plant'],sprintf('Plant variant/%d',statePort),'DroneBus/1');
add_line([mdl '/Selected plant'],sprintf('Plant variant/%d',3-statePort),'DroneData/1');
% If the selected instance is itself a Subsystem Reference, fork its files
% before retargeting any child model references. Originals remain untouched.
[subsystemCopies,~]=cloneSubsystemReferences([mdl '/Selected plant'],here,ref,0);
% Make a private analysis copy; never change the original referenced model.
refs=find_system([mdl '/Selected plant'],'LookUnderMasks','all', ...
    'MatchFilter',@Simulink.match.allVariants,'BlockType','ModelReference');
for k=1:numel(refs)
    original=which(string(get_param(refs{k},'ModelName'))+".slx");
    % Isolate every referenced model; local solver settings change only copies.
    if k==1, copyName=analysis; else, copyName=[analysis '_' num2str(k)]; end
    copyfile(original,fullfile(here,[copyName '.slx']));
    load_system(fullfile(here,[copyName '.slx']));
    solvers=find_system(copyName,'LookUnderMasks','all','FollowLinks','off', ...
        'MatchFilter',@Simulink.match.allVariants,'RegExp','on', ...
        'ReferenceBlock','^nesl_utility/Solver');
    if isempty(solvers)
        solvers=find_system(copyName,'LookUnderMasks','all','FollowLinks','off', ...
            'MatchFilter',@Simulink.match.allVariants,'RegExp','on', ...
            'Name','^Solver\s*Configuration$');
    end
    for j=1:numel(solvers)
        set_param(solvers{j},'UseLocalSolver','off','MultibodyUseLocalSolver','off');
    end
    set_param(copyName,'SolverType','Variable-step','Solver','ode23t');
    set_param(copyName,'SimMechanicsOpenEditorOnUpdate','off');
    joint=[copyName '/6DOF/6-DOF Joint'];
    if getSimulinkBlockHandle(joint)>0
        % Initial targets belong only to the analysis copy. They do not lock
        % the joint or change its free-flight equations of motion.
        axisNames=["Px","Py","Pz"];
        for axis=1:3
            prefix=char(axisNames(axis));
            set_param(joint,[prefix 'PositionTargetSpecify'],'on', ...
                [prefix 'PositionTargetValue'],num2str(hoverPosition(axis),17), ...
                [prefix 'VelocityTargetSpecify'],'on',[prefix 'VelocityTargetValue'],'0');
        end
        set_param(joint,'SphPositionTargetSpecify',char(matlab.lang.OnOffSwitchState(levelHover)), ...
            'SphPositionTargetRotationMethod','Quaternion', ...
            'SphPositionTargetQuaternion',mat2str([cos(yaw/2) 0 0 sin(yaw/2)],17), ...
            'SphVelocityTargetSpecify','on','SphVelocityTargetValue','[0 0 0]');
    end
    speeds=evalin('base','IC.MotorOmega');
    params=evalin('base','SimParams');
    for motor=1:numel(speeds)
        direction=1;
        inverter=sprintf('%s/Motor %d/Gain',copyName,motor);
        if getSimulinkBlockHandle(inverter)>0
            gain=str2double(get_param(inverter,'Gain'));
            if isfinite(gain) && gain~=0, direction=sign(gain); end
        end
        rotor=sprintf('%s/Propeller %d/Revolute Joint',copyName,motor);
        if getSimulinkBlockHandle(rotor)>0
            set_param(rotor,'VelocityTargetSpecify','on', ...
                'VelocityTargetValue',num2str(direction*speeds(motor),17), ...
                'VelocityTargetValueUnits','rad/s');
        end
        controller=sprintf('%s/Motor %d/Thrust Controller',copyName,motor);
        if getSimulinkBlockHandle(controller)>0 && isfield(params,'kQ')
            qCoefficient=params.kQ;
            if numel(qCoefficient)>1, qCoefficient=qCoefficient(motor); end
            set_param(controller,'InitialConditionForIntegrator', ...
                num2str(direction*qCoefficient*speeds(motor)^2,17));
        end
    end
    storeContext(copyName);
    save_system(copyName);
    set_param(refs{k},'ModelName',copyName);
end
for k=numel(subsystemCopies):-1:1, save_system(subsystemCopies{k}); end

oldFolder = pwd;
guard = onCleanup(@()cd(oldFolder));
cd(here);
new_system(ref,'SubSystem');
Simulink.SubSystem.copyContentsToBlockDiagram([mdl '/Selected plant'],ref);
save_system(ref,fullfile(here,[ref '.slx']));
Simulink.SubSystem.deleteContents([mdl '/Selected plant']);
set_param([mdl '/Selected plant'],'ReferencedSubsystem',ref);
add_block('simulink/Signal Routing/Bus Selector',[mdl '/Flight signals'], ...
    'OutputSignals',char(strjoin(stateSignals,',')), ...
    'Position',[600 130 610 330]);
add_block('simulink/Signal Routing/Mux',[mdl '/Flight state vector'], ...
    'Inputs','4','Position',[815 140 820 260]);
add_block('simulink/Sinks/Out1',[mdl '/Flight states NED FRD'], ...
    'Port','1','Position',[900 185 930 205]);
add_block('simulink/Sinks/Terminator',[mdl '/Auxiliary data'], ...
    'Position',[585 375 605 395]);
add_line(mdl,'Motor perturbation/1','Motor command/2','autorouting','on');
add_line(mdl,'Hover command/1','Motor command/1','autorouting','on');
add_line(mdl,'Motor command/1','Selected plant/1','autorouting','on');
% Variant subsystem output 1 is DroneBus; output 2 is auxiliary DroneData.
add_line(mdl,'Selected plant/1','Flight signals/1','autorouting','on');
add_line(mdl,'Selected plant/2','Auxiliary data/1','autorouting','on');
for k=1:4
    add_line(mdl,sprintf('Flight signals/%d',k), ...
        sprintf('Flight state vector/%d',k),'autorouting','on');
end
add_line(mdl,'Flight state vector/1','Flight states NED FRD/1','autorouting','on');
io(1)=linio([mdl '/Motor perturbation'],1,'input');
io(2)=linio([mdl '/Flight state vector'],1,'output');
setlinio(mdl,io);
a=Simulink.Annotation(mdl, sprintf(['HOVER LINEARISATION | selected plant: Config.Plant.Type\n' ...
    '1. Initialise the hardware selection in Main (no PX4 startup needed).\n' ...
    '2. Run linearize_drone: known hover input, one snapshot, then A/B/C/D.\n' ...
    'Inputs: delta normalised motor commands about the known hover values.\n' ...
    'States: S_NED; Sdot_NED; Eta_NED; Omega_nav_ned (12 variables).\n' ...
    'The selected plant is a Subsystem Reference; double-click to inspect.']));
a.Position=[45 420 920 565];
save_system(mdl,fullfile(here,[mdl '.slx']));
fprintf('Referenced plant harness created. MATLAB will compile it for linearisation.\n');
end

function prepareParameters(Config,hoverInput)
SimParams=evalin('base','SimParams');
assignin('base','TopPayloadVariant',Config.TopPayload.VariantValue);
if ~evalin('base','exist(''Aero_On'',''var'')'), assignin('base','Aero_On',1); end
if ~evalin('base','exist(''CommParams'',''var'')')
    assignin('base','CommParams',communication_parameters());
end
if isempty(hoverInput)
    if isfield(SimParams,'HoverMotorCommand')
        hoverInput=SimParams.HoverMotorCommand(:);
    else
        assert(isfield(SimParams,'RotorPositions'),'Pass HoverInput for a custom actuator interface.');
        n=size(SimParams.RotorPositions,1);
        omega=sqrt(SimParams.HoverThrust./(n*SimParams.kT));
        hoverInput=omega./SimParams.CommandMaxOmega;
        if isscalar(hoverInput), hoverInput=repmat(hoverInput,n,1); end
    end
end
assignin('base','LinHoverCommand',hoverInput(:));
if string(Config.Plant.Type)=="Simscape" && ~evalin('base','exist(''SimscapeParams'',''var'')')
    assert(evalin('base','exist(''Vehicle'',''var'')==1'),'Run the Main vehicle initialisation first.');
    Vehicle=evalin('base','Vehicle');
    assignin('base','SimscapeParams',build_simscape_parameters(Config,Vehicle,SimParams));
end
assert(all(evalin('base','LinHoverCommand')>0 & evalin('base','LinHoverCommand')<1), ...
    'Selected propulsion cannot hover within motor-command bounds.');
end


function restoreWorkspace(names,existed,saved)
for k=1:numel(names)
    if existed(k), assignin('base',names{k},saved{k});
    else, evalin('base',['clear ' names{k}]); end
end
end

function closeAnalysisModels(models)
loaded=find_system('type','block_diagram');
for k=1:numel(loaded)
    if any(startsWith(string(loaded{k}),string(models)))
        close_system(loaded{k},0);
    end
end
end

function [copies,count]=cloneSubsystemReferences(scope,folder,prefix,count)
copies={};
blocks=find_system(scope,'LookUnderMasks','all','FollowLinks','off', ...
    'LookInsideSubsystemReference','off','MatchFilter',@Simulink.match.allVariants, ...
    'BlockType','SubSystem');
for k=1:numel(blocks)
    original=get_param(blocks{k},'ReferencedSubsystem');
    if isempty(original), continue; end
    resolved=which(original);
    if isempty(resolved), resolved=which(string(original)+".slx"); end
    assert(~isempty(resolved),'Cannot locate referenced subsystem %s.',original);
    count=count+1;
    name=[prefix '_source' num2str(count)];
    copyfile(resolved,fullfile(folder,[name '.slx']));
    load_system(fullfile(folder,[name '.slx']));
    set_param(blocks{k},'ReferencedSubsystem',name);
    [children,count]=cloneSubsystemReferences(name,folder,prefix,count);
    copies=[copies,{name},children]; %#ok<AGROW>
end
end

function storeContext(model)
% Persist the selected Main configuration inside generated analysis files.
mw=get_param(model,'ModelWorkspace');
vars={'Config','Vehicle','SimParams','SimscapeParams','CommParams','IC', ...
    'Aero_On','TopPayloadVariant','LinHoverCommand'};
for k=1:numel(vars)
    if evalin('base',sprintf('exist(''%s'',''var'')==1',vars{k}))
        assignin(mw,vars{k},evalin('base',vars{k}));
    end
end
end
