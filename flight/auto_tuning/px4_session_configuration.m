function [cfg,Stages]=px4_session_configuration(request,options)
%PX4_SESSION_CONFIGURATION Stages and settings of a single-loop tuning session (run_px4_tuning).
%   [cfg,Stages] = px4_session_configuration(request,options) returns the selected stages ("all" or
%   stage keys such as "rollrate"), each with its PX4 mode, axis, step amplitude, searched and fixed
%   gains, and the session settings cfg: models, simulation time, test start (40 s), step counts,
%   trial limit, wall budget, hover altitude (10 m) and the safety limits.
allStages=[stage("rollrate","Roll rate",1,1,0.15,{'MC_ROLLRATE_D','MC_ROLLRATE_P','MC_ROLLRATE_I'},{'MC_ROLLRATE_K'}), ...
    stage("pitchrate","Pitch rate",1,2,0.15,{'MC_PITCHRATE_D','MC_PITCHRATE_P','MC_PITCHRATE_I'},{'MC_PITCHRATE_K'}), ...
    stage("yawrate","Yaw rate",1,3,0.15,{'MC_YAWRATE_P','MC_YAWRATE_I','MC_YAWRATE_D'},{'MC_YAWRATE_K'}), ...
    stage("rollattitude","Roll attitude",2,1,0.10,{'MC_ROLL_P'},{}), ...
    stage("pitchattitude","Pitch attitude",2,2,0.10,{'MC_PITCH_P'},{}), ...
    stage("yawattitude","Yaw attitude",2,3,0.10,{'MC_YAW_P'},{}), ...
    stage("verticalvelocity","Vertical velocity",3,3,0.60,{'MPC_Z_VEL_P_ACC','MPC_Z_VEL_D_ACC','MPC_Z_VEL_I_ACC'},{}), ...
    stage("horizontalvelocity","Horizontal velocity",3,1,0.80,{'MPC_XY_VEL_P_ACC','MPC_XY_VEL_D_ACC','MPC_XY_VEL_I_ACC'},{}), ...
    stage("verticalposition","Vertical position",4,3,0.60,{'MPC_Z_P'},{}), ...
    stage("horizontalposition","Horizontal position",4,1,0.80,{'MPC_XY_P'},{})];
if isequal(request,"all"),Stages=allStages;else
    keys=[allStages.Key];Stages=allStages([]);
    for key=lower(request)
        index=find(keys==key);assert(isscalar(index),'Unknown tuning stage: %s.',key);
        Stages(end+1)=allStages(index); %#ok<AGROW>
    end
end
assert(~isempty(Stages),'Select at least one stage.');
limit=options.MaxTrials;
if limit==0
    if options.Mode=="verify",limit=3;elseif options.Mode=="validate",limit=1;else,limit=30;end
end
if options.Mode=="validate",assert(limit==1,'Validation uses one baseline per loop.');end
if options.Mode=="verify"
    assert(numel(Stages)==1&&limit<=3,'Verification is one stage and at most three trials.');
end
for k=1:numel(Stages),Stages(k).MaxTrials=limit;end
cfg=struct('Plant','Drone_simulation_PX4','PX4Model','PX4_Receive', ...
    'RunMode',options.Mode,'SessionMode',true,'StageLimit',numel(Stages), ...
    'SimTime',110,'TestStart',40,'ExpectedSteps',10,'MinimumSteps',5,'MaxTrials',limit,'BudgetSec',16*3600, ...
    'ObjectiveVersion',"joint_response_v1",'MaxWallPerRun',1800,'MinRemainingToStart',60,'HoverThrust',single(hoverThrust()), ...
    'HomeNED',single([0 0 -10]),'MinTestAltitude',2,'MinAltitude',2, ...
    'MaxReadyClimbRate',1,'MaxBodyRate',5,'MaxRecoveryRate',5, ...
    'MinimumImprovement',options.MinimumImprovement, ...
    'StagnationTrials',5,'ShowFigures',options.ShowFigures);
for k=1:numel(Stages)
    [~,~,duration]=px4_tuning_step_windows(Stages(k).Mode);
    Stages(k).SimTime=10*ceil((cfg.TestStart+30+duration+5)/10);
end
cfg.SimTime=max([Stages.SimTime]);
if isfield(options,'MaxHours'),cfg.BudgetSec=options.MaxHours*3600;end
cfg.FlightPlan=flight_plan("square");cfg.FlightPlan.Size=0;
cfg.FlightPlan.Altitude=-double(cfg.HomeNED(3));
end
function S=stage(key,name,mode,axis,amplitude,parameters,fixed)
S=struct('Key',string(key),'Name',string(name),'Mode',uint8(mode),'Axis',uint8(axis), ...
    'Amplitude',single(amplitude),'Parameters',{parameters},'FixedParameters',{fixed},'MaxTrials',3);
end

function h=hoverThrust()
% Normalised hover thrust of the current vehicle (PX4 thrust command with THR_MDL_FAC = 1), from the
% workspace built by Main.mlx; 0.3 if no vehicle has been built yet.
h=0.3;
try,SP=evalin('base','SimParams');h=SP.HoverThrustFraction;catch,end
end
