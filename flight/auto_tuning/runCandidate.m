function [M,status,wallSec,diagnosis] = runCandidate(cfg,S,tune,totalClock)
%RUNCANDIDATE Run and score one PX4 tuning candidate from fresh SDI data.
% Starts the Simscape plant before PX4. Does not read or modify PX4 ULogs.
% Candidates are scored from simulation data without target parameter readback.

M = emptyMetrics();
status = "ERROR: not started";
watcher = [];
priorIds = Simulink.sdi.getAllRunIDs;
started = tic;
cleanupOK = false;
simulationOK = false;
trialDir=char(cfg.OutputDirectory);
if ~isfield(cfg,'TrialPrefix'),cfg.TrialPrefix=['Trial_' char(datetime('now','Format','yyyyMMdd_HHmmss_SSS')) '_'];end
prefix=char(cfg.TrialPrefix);
completedSimTime=0;
plantData=[];controllerData=[];
try
    model = char(cfg.PX4Model);
    plant = char(cfg.Plant);
    assert(bdIsLoaded(model) && bdIsLoaded(plant), ...
        'Load both models before starting the candidate runner.');
    assertTuningBranchEnabled(model);
    assert(strcmp(codertarget.data.getParameterValue(model,'Runtime.BuildAction'), ...
        'Build, load and run'), ...
        'PX4 must use MATLAB Build, load and run for each candidate.');
    tick=str2double(get_param(Simulink.ID.getHandle([model ':341']),'SampleTime'));
    assert(isfinite(tick)&&tick<=0.1, ...
        'Parameter-write clock is too coarse for its 0.5-to-1-second trigger window. Reload the repaired saved model.');

    % Reject an old loaded model rather than silently testing at the wrong height.
    stateflowRoot=sfroot;charts=stateflowRoot.find('-isa','Stateflow.EMChart');consistentAltitude=false;
    for chartIndex=1:numel(charts)
        if startsWith(charts(chartIndex).Path,[model '/']) && ...
                contains(charts(chartIndex).Script,'function thrust = test_altitude_thrust')
            consistentAltitude=contains(charts(chartIndex).Script,'h_ref = -double(HomeNED(3));');
        end
    end
    assert(consistentAltitude,'Reload the saved model: altitude harness must follow HomeNED.');
    prepare_px4_tuning_workspace(cfg,S,tune);
    configure_px4_receive_for_batch(model);
    assert(getSimulinkBlockHandle([model '/Mission Publishers'])~=-1, ...
        'Install the mutually exclusive mission/tuning publishers before tuning.');
    snapshot=struct('FlightPlan',cfg.FlightPlan,'Stage',S,'PX4Tune',tune, ...
        'TestStart',cfg.TestStart,'CurrentFolder',pwd, ...
        'ControllerModel',get_param(model,'FileName'), ...
        'PlantModel',get_param(plant,'FileName'));
    save(fullfile(trialDir,[prefix 'CandidateSetup.mat']),'snapshot');

    set_param(plant,'StopTime','inf','SignalLogging','on');
    set_param(model,'StopTime',num2str(cfg.SimTime), ...
        'ReturnWorkspaceOutputs','off');
    watcher = px4_ack_popup(min(cfg.MaxWallPerRun, ...
        max(60,cfg.BudgetSec-toc(totalClock))));

    % The plant owns the server endpoint; start it before the PX4 client.
    set_param(plant,'SimulationCommand','start');
    waitForPlant(plant,60);
    set_param(model,'SimulationCommand','start');
    completedSimTime=waitForPX4(model,cfg.SimTime, ...
        min(cfg.MaxWallPerRun,max(60,cfg.BudgetSec-toc(totalClock))));
    simulationOK = true;
    stop_px4_run(watcher);
    watcher = [];
    cleanupOK = true;
catch ME
    status = "ERROR: " + string(ME.message);
    reportFile=fullfile(trialDir,[prefix 'ExecutionError.txt']);
    fid=fopen(reportFile,'wt');
    if fid~=-1,fprintf(fid,'%s',getReport(ME,'extended','hyperlinks','off'));fclose(fid);end
    if exist('plant','var') && bdIsLoaded(plant) && ...
            ~strcmp(get_param(plant,'SimulationStatus'),'stopped') || ~isempty(watcher)
        try
            stop_px4_run(watcher);
            cleanupOK = true;
        catch cleanupError
            status = status + " | cleanup failed: " + string(cleanupError.message);
        end
    end
end
wallSec = toc(started);
if ~simulationOK || ~cleanupOK
    try
        ids=setdiff(Simulink.sdi.getAllRunIDs,priorIds);
        [plantData,controllerData]=exportFreshRuns(ids,plant,model,trialDir,prefix);
    catch
        % Any available partial datasets were saved before completeness checks.
    end
    M.SimulationTime=completedSimTime;
diagnosis=write_px4_trial_report(plantData,controllerData,S,cfg,status,trialDir,tune);
    printResult(M,status,wallSec);
    return;
end

if isfield(S,'Members')
    % Multi-axis flight: score and report every axis segment separately.
    [M,status,diagnosis]=scoreMembers(S,cfg,tune,ids0(priorIds),plant,model,trialDir,prefix,completedSimTime,wallSec);
    return;
end
try
    ids = setdiff(Simulink.sdi.getAllRunIDs,priorIds);
    [plantData,controllerData] = exportFreshRuns(ids,plant,model,trialDir,prefix);
    trigger=controllerData.getElement('PX4_PARAMETER_WRITE_TRIGGER').Values;
    assert(any(trigger.Data~=0,'all'), ...
        'Parameter-write trigger never fired; candidate gains cannot be treated as applied.');
    M = score_px4_tuning_candidate(plantData,controllerData,S,cfg);
    status = "VALID";
    if isfield(M,'JointFeatures')
        Objective=M.JointFeatures;save(fullfile(trialDir,[prefix 'Objective.mat']),'Objective');
    end
catch ME
    status = "INVALID: " + string(ME.message);
end
M.SimulationTime=completedSimTime;
diagnosis=write_px4_trial_report(plantData,controllerData,S,cfg,status,trialDir,tune);
fprintf('MATLAB trial report: %s\n',fullfile(trialDir,diagnosis.ReportFile));
printResult(M,status,wallSec);
end

function ids=ids0(priorIds)
ids=setdiff(Simulink.sdi.getAllRunIDs,priorIds);
end

function [M,status,diagnosis]=scoreMembers(S,cfg,tune,ids,plant,model,trialDir,prefix,simTime,wallSec)
% Returns cell arrays M{k}, diagnosis{k} and status(k), one per axis segment.
n=numel(S.Members);status=strings(1,n);M=cell(1,n);diagnosis=cell(1,n);
plantData=[];controllerData=[];common="";
try
    [plantData,controllerData]=exportFreshRuns(ids,plant,model,trialDir,prefix);
    trigger=controllerData.getElement('PX4_PARAMETER_WRITE_TRIGGER').Values;
    assert(any(trigger.Data~=0,'all'), ...
        'Parameter-write trigger never fired; candidate gains cannot be treated as applied.');
catch ME
    common="INVALID: "+string(ME.message);
end
for k=1:n
    Sk=S.Members(k);cfgk=cfg;cfgk.TrialPrefix=[prefix char(Sk.Key) '_'];
    Mk=emptyMetrics();
    if common==""
        try
            Mk=score_px4_tuning_candidate(plantData,controllerData,Sk,cfg);status(k)="VALID";
            if isfield(Mk,'JointFeatures')
                Objective=Mk.JointFeatures;save(fullfile(trialDir,[cfgk.TrialPrefix 'Objective.mat']),'Objective');
            end
        catch ME
            status(k)="INVALID: "+string(ME.message);
        end
    else
        status(k)=common;
    end
    Mk.SimulationTime=simTime;Mk.Axis=double(Sk.Axis);Mk.Key=string(Sk.Key);
    dk=write_px4_trial_report(plantData,controllerData,Sk,cfgk,status(k),trialDir,tune);
    M{k}=Mk;diagnosis{k}=dk;
    printResult(Mk,Sk.Name+": "+status(k),wallSec);
end
end

function assertTuningBranchEnabled(model)
root = find_system(model,'SearchDepth',1,'BlockType','Constant');
hasTuneConstant = false;
for k=1:numel(root)
    if strcmp(strtrim(get_param(root{k},'Value')),'TUNING_ON') && ...
            strcmp(get_param(root{k},'Commented'),'off')
        hasTuneConstant = true;
    end
end
assert(hasTuneConstant, ...
    'Uncomment the root TUNING_ON Constant before building the tuning model.');
blocks = find_system(model,'SearchDepth',1,'BlockType','SubSystem');
branch = {};
for k=1:numel(blocks)
    compactName=regexprep(strtrim(get_param(blocks{k},'Name')),'\s+','');
    if strcmp(compactName,'EnabledSubsystem'), branch{end+1}=blocks{k}; end %#ok<AGROW>
end
assert(~isempty(branch) && strcmp(get_param(branch{1},'Commented'),'off'), ...
    'Uncomment the Enabled Subsystem that contains the tuning path.');
end

function waitForPlant(plant,timeout)
t=tic;
while ~strcmp(get_param(plant,'SimulationStatus'),'running')
    assert(toc(t)<timeout,'Plant did not start within %.0f seconds.',timeout);
    pause(0.5);
end
end

function completedTime=waitForPX4(model,simTime,timeout)
t=tic;
seenRunning=false;lastProgress=-inf;lastModelTime=-inf;lastAdvance=0;
while true
    assert(toc(t)<timeout,'PX4 run exceeded %.0f seconds wall time.',timeout);
    connected = strcmp(get_param(model,'ExtModeConnected'),'on');
    state = string(get_param(model,'SimulationStatus'));
    raw = get_param(model,'SimulationTime');
    simNow = str2double(string(raw));
    seenRunning=seenRunning || connected || simNow>0;
    if isfinite(simNow)&&simNow>lastModelTime+1e-6
        lastModelTime=simNow;lastAdvance=toc(t);
    end
    assert(~seenRunning||toc(t)-lastAdvance<120, ...
        'PX4 model time stalled at %.3f s for 120 wall seconds; execution must be diagnosed.',simNow);
    if simNow-lastProgress>=10
        fprintf('PX4 simulation: %.1f / %.0f s\n',simNow,simTime);lastProgress=simNow;
    end
    if ~seenRunning && state=="stopped" && toc(t)<120
        pause(1); continue;
    end
    assert(state~="stopped" || simNow>=simTime-0.05, ...
        'PX4 stopped early at %.3f simulated seconds.',simNow);
    if simNow>=simTime-0.05
        completedTime=simNow;
        return;
    end
    assert(connected || state=="running" || state=="external", ...
        'PX4 external-mode connection was lost during the run.');
    pause(1);
end
end

function [plantData,controllerData] = exportFreshRuns(ids,plantName,controllerName,outDir,prefix)
plantData=[]; controllerData=[];
for k=1:numel(ids)
    r=Simulink.sdi.getRun(ids(k));
    d=Simulink.sdi.exportRun(ids(k));
    n=string(r.Name);
    safe=matlab.lang.makeValidName(char(n));
    save(fullfile(outDir,sprintf('%s%s_%d_sdi.mat',prefix,safe,ids(k))),'d','-v7.3');
    if contains(n,plantName), plantData=d; end
    if contains(n,controllerName), controllerData=d; end
end
assert(~isempty(plantData),'No fresh Simscape plant SDI run was captured.');
assert(~isempty(controllerData),'No fresh PX4 controller SDI run was captured.');
end

function M=emptyMetrics()
M=struct('Score',inf,'RMSE',NaN,'Overshoot',NaN, ...
    'MinAltitude',NaN,'MaxRate',NaN,'RecoveryPeak',NaN,'SimulationTime',0,'StepCount',0);
end

function printResult(M,status,wall)
fprintf('Candidate result: %s | score %.6g | usable steps %d/10 (minimum 5) | wall %.1f s\n',status,M.Score,M.StepCount,wall);
end
