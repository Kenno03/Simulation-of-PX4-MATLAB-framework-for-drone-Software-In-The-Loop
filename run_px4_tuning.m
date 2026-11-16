function Session=run_px4_tuning(stageNames,options)
%RUN_PX4_TUNING Tuning session for single PX4 loops (one loop per flight), or the menu entry point.
%   Session = run_px4_tuning()                                   numbered menu (px4_tuning_menu)
%   Session = run_px4_tuning("rollrate",Mode="verify")           short check, at most 3 trials
%   Session = run_px4_tuning(["rollrate","pitchrate"],Mode="validate")   fly the current gains once
%   Session = run_px4_tuning("all",Mode="tune",MaxTrials=30)     joint finite-difference search
%   Session = run_px4_tuning(ResumeFolder="...")                 continue after an execution fault
%   Menu entries 17 and 18 start the multi-axis tuner run_px4_guide_tuning (guide or RMSE method,
%   at most 24 h). Every session is saved in TuningSessions/<timestamp>; reload it later with
%   load_px4_tuning_session(Session.Folder). Select the Default profile in Main.mlx before tuning.
arguments
    stageNames (1,:) string = strings(1,0)
    options.Mode (1,1) string {mustBeMember(options.Mode,["verify","validate","tune"])} = "verify"
    options.MaxHours (1,1) double {mustBePositive,mustBeFinite} = 16
    options.MaxTrials (1,1) double {mustBeInteger,mustBeNonnegative} = 0
    options.ShowFigures (1,1) logical = true
    options.OutputRoot (1,1) string = ""
    options.ResumeFolder (1,1) string = ""
    options.MinimumImprovement (1,1) double {mustBePositive} = 0.01
end
project=fileparts(mfilename('fullpath'));
addpath(project);project_paths(project);
prepare_px4_tuning_paths();
resuming=options.ResumeFolder~="";
assert(~resuming||isempty(stageNames),'Resume uses the saved stage list.');
if isempty(stageNames)&&~resuming
    [stageNames,mode,limit]=px4_tuning_menu();
    if isempty(stageNames)
        Session=struct('Status',"CANCELLED",'Folder',"");return;
    end
    if isequal(stageNames,"guide")||isequal(stageNames,"rmse")
        % Multi-axis tuning of all loop groups; see run_px4_guide_tuning (guide rules or RMSE search).
        Session=run_px4_guide_tuning("all",Method=stageNames,MaxHours=min(options.MaxHours,24), ...
            ShowFigures=options.ShowFigures,OutputRoot=options.OutputRoot);
        return;
    end
    options.Mode=mode;options.MaxTrials=limit;
end
px4_session_lock("acquire");
lockGuard=onCleanup(@()px4_session_lock("release")); %#ok<NASGU>
if resuming
    [Session,cfg,Stages,firstStage]=restore_px4_tuning_checkpoint(options.ResumeFolder);
    folder=Session.Folder;
else
    [cfg,Stages]=px4_session_configuration(stageNames,options);
    if options.OutputRoot=="",root=fullfile(project,'TuningSessions');else,root=options.OutputRoot;end
    if ~isfolder(root),mkdir(root);end
    scope=Stages(1).Key;
    if numel(Stages)>1,scope=string(numel(Stages))+"loops";end
    % Keep group paths short enough for MATLAB/Windows artifact filenames.
    folder=fullfile(root,"PX4_"+string(datetime('now','Format','yyyyMMdd_HHmmss_SSS'))+"_"+scope);
    assert(~isfolder(folder),'A session folder with this timestamp already exists.');
    mkdir(folder);cfg.OutputDirectory=char(folder);
    Session=struct('Version',3,'Folder',string(folder),'StartedAt',datetime('now'), ...
        'FinishedAt',NaT,'Status',"INITIALIZING",'Config',cfg,'Stages',Stages, ...
        'Results',emptyResults(),'GainHistory',emptyHistory(),'StageResults',table(), ...
        'JointHistory',table(),'Quality',table(), ...
        'StartGains',struct(),'BestGains',struct(),'Initialization',struct(), ...
        'FigureFiles',strings(0,1),'ReportFiles',strings(0,1),'FatalError',"");
    firstStage=1;
    Session.OriginalBudgetSec=cfg.BudgetSec;
    Session.WallDeadline=Session.StartedAt+seconds(cfg.BudgetSec);
end
diary(fullfile(folder,'console_log.txt'));
try,sldiagviewer.diary(fullfile(folder,'simulink_diagnostics.txt'));catch,end
cleanup=onCleanup(@()px4_session_cleanup(char(folder),{cfg.PX4Model,cfg.Plant}));
fprintf('MATLAB tuning session: %s\nMode: %s | max trials/stage: %d | filters fixed\n',folder,cfg.RunMode,cfg.MaxTrials);
clock=tic;
try
    checkpoint();
    if resuming
        names=fieldnames(Session.Initialization);
        for k=1:numel(names),assignin('base',names{k},Session.Initialization.(names{k}));end
        tune=Session.BestGains;
        assignin('base','PX4Tune',tune);
    else
        ensureInitialization(project);
        names={'Config','Vehicle','TopPayloadVariant','SimParams','SimscapeParams', ...
            'CommParams','IC','Aero_On','PX4Tune','PX4Params'};
        for k=1:numel(names)
            Session.Initialization.(names{k})=evalin('base',names{k});
        end
        tune=Session.Initialization.PX4Tune;
    end
    assert(strcmp(string(tune.ID),"Default"),'Select Default in Main before tuning.');
    defaults=px4_tuning_parameters("Default");
    for name=px4_controller_gain_names()
        if ~isfield(tune,name),tune.(name)=defaults.(name);end
    end
    if ~resuming
        if cfg.RunMode~="validate",tune=px4_default_seed(tune,defaults);end
        Session.Initialization.PX4Tune=tune;assignin('base','PX4Tune',tune);
        Session.StartGains=tune;Session.BestGains=tune;
        Session.SearchVersion="joint_fd_v1";Session.SeedSource="Default";
        if cfg.RunMode=="validate",Session.SeedSource="Caller profile; validation only";end
        Session.JointHistory=table();Session.Quality=table();
        Session.AcceptancePolicy="distinct_gain_update_v1";
    end
    for model={cfg.PX4Model,cfg.Plant}
        if bdIsLoaded(model{1})&&~strcmp(get_param(model{1},'SimulationStatus'),'stopped')
            stop_px4_run();break;
        end
    end
    prepare_px4_tuning_workspace(cfg,Stages(firstStage),tune);
    load_system(fullfile(project,'PX4_Receive.slx'));
    load_system(fullfile(project,'Drone Models','Drone_simulation_PX4.slx'));
    load_system(fullfile(project,'Drone Models','Simscape_Drone_Model.slx'));
    % A call takes ownership of the selected project's running models.
    if ~strcmp(get_param(cfg.Plant,'SimulationStatus'),'stopped') || ...
            ~strcmp(get_param(cfg.PX4Model,'SimulationStatus'),'stopped')
        stop_px4_run();
    end
    assert(getSimulinkBlockHandle('PX4_Receive/Parameter Set/Roll Rate Tuning Parameters')==-1, ...
        'Reload the consolidated saved model; the obsolete roll tuning subsystem is still loaded.');
    configure_px4_receive_for_batch(cfg.PX4Model);
    cfg.ShowFigures=false; % Save each figure once; display the final session in MATLAB.
    cfg.SessionMode=true;
    Session.Config=cfg;
    Session.Status="RUNNING";
    for stageIndex=firstStage:numel(Stages)
        S=Stages(stageIndex);cfg.SimTime=S.SimTime;Session.CurrentStage=string(S.Key);initialAmplitude=double(S.Amplitude);
        if resuming&&stageIndex==firstStage
            state=Session.StageState;search=Session.SearchState;
            attempts=Session.StageTrialCount;
            validCount=nnz(Session.Results.Stage==S.Name&Session.Results.Status=="VALID");
            S.Amplitude=single(Session.CurrentAmplitude);
            baseline=~state.HasBaseline;stopReason="TRIAL LIMIT";
            fprintf('Continuing %s after %d attempts; completed stages retained.\n',S.Name,attempts);
            checkpoint();
        else
            state=struct('BestScore',inf,'BestTune',Session.BestGains, ...
                'StagnantValidTrials',0,'HasBaseline',false,'BestUsable',false);
            Session.GainHistory=historyRows(Session.GainHistory,0,S,state.BestTune,state.BestTune,"INITIAL",NaN);
            checkpoint();
            attempts=0;validCount=0;stopReason="TRIAL LIMIT";
            baseline=true;
            search=px4_joint_initialize(S,defaults);
        end
        while attempts<cfg.MaxTrials && toc(clock)<cfg.BudgetSec
            if isfile(fullfile(cfg.OutputDirectory,'pause.request'))
                Session.Status="PAUSED";checkpoint();
                waited=px4_tuning_wait_for_resume(cfg.OutputDirectory);
                cfg.BudgetSec=cfg.BudgetSec+waited;Session.Config.BudgetSec=cfg.BudgetSec;
                Session.WallDeadline=Session.WallDeadline+seconds(waited);
                Session.Status="RUNNING";checkpoint();
            end
            if baseline
                candidate=state.BestTune;parameter="Baseline";value=NaN;changed=strings(1,0);
            else
                [search,candidate,parameter,changed]=px4_joint_next(search,state.BestTune,state.BestScore);
                value=NaN;
            end
            attempts=attempts+1;trialNumber=height(Session.Results)+1;
            cfg.TrialPrefix=sprintf('Trial%03d_%s_',trialNumber,S.Key);
            fprintf('\n%s trial %d/%d | %s | excitation %.5g\n',S.Name,attempts,cfg.MaxTrials,parameter,S.Amplitude);
            [M,status,wall,diagnosis]=runCandidate(cfg,S,candidate,clock);
            valid=status=="VALID";
            [state,accepted,improvement]=px4_tuning_accept_trial(state,candidate,M.Score,valid,cfg.MinimumImprovement);
            if valid,validCount=validCount+1;end
            gainFailure=~valid&&any(string(diagnosis.Code)==["EXCITATION_OR_RECOVERY_UNSTABLE","BASELINE_UNSTABLE"]);
            if ~baseline&&(valid||gainFailure)
                search=px4_joint_observe(search,M.Score,valid,accepted);
            end
            usable=valid&&isfield(M,'JointFeatures')&&M.JointFeatures.Usable;
            if accepted,state.BestUsable=usable;end
            jointRow=table(trialNumber,string(S.Name),parameter,strjoin(changed,','), ...
                {candidate},M.Score,valid,accepted,usable,'VariableNames', ...
                {'Run','Stage','Proposal','ChangedGains','GainSet','Score','Valid','Accepted','Usable'});
            for feature=["Tracking","LateError","OvershootRMS","Oscillation","MotorVariation"]
                jointRow.(feature)=NaN;
                if valid&&isfield(M,'JointFeatures'),jointRow.(feature)=M.JointFeatures.(feature);end
            end
            if isempty(Session.JointHistory),Session.JointHistory=jointRow;
            else,Session.JointHistory=[Session.JointHistory;jointRow];end
            Session.BestGains=state.BestTune;
            label="INVALID";
            if valid&&baseline,label="BASELINE";elseif accepted,label="ACCEPTED";elseif valid,label="REJECTED";end
            Session.GainHistory=historyRows(Session.GainHistory,trialNumber,S,candidate,state.BestTune,label,M.Score);
            row=table(trialNumber,string(S.Name),parameter,value,double(S.Amplitude), ...
                M.Score,state.BestScore,M.RMSE,M.Overshoot,M.MinAltitude,M.MaxRate,M.RecoveryPeak,wall, ...
                status,string(diagnosis.Code),accepted,improvement,state.StagnantValidTrials,string(cfg.TrialPrefix),M.SimulationTime,M.StepCount, ...
                'VariableNames',Session.Results.Properties.VariableNames);
            Session.Results=[Session.Results;row];
            Session.ReportFiles(end+1)=string(diagnosis.ReportFile);
            Session.FigureFiles=unique([Session.FigureFiles;string(diagnosis.FigureFiles(:))],'stable');
            fprintf('Diagnosis: %s\n%s\n',diagnosis.Code,diagnosis.Summary);
            Session.SearchState=search;Session.StageState=state;
            Session.StageTrialCount=attempts;Session.CurrentAmplitude=S.Amplitude;
            checkpoint();
            if gainFailure&&~baseline
                fprintf('Infeasible gain vector retained as evidence; accepted gains unchanged.\n');
                continue;
            end
            if ~valid
                if diagnosis.AllowAmplitudeRetry && attempts<cfg.MaxTrials && ...
                        double(S.Amplitude)*0.5>=initialAmplitude*0.25
                    S.Amplitude=single(double(S.Amplitude)*0.5);
                    state.BestScore=inf;state.HasBaseline=false;state.StagnantValidTrials=0;
                    baseline=true;
                    search=px4_joint_initialize(S,defaults);
                    fprintf('Diagnostic retry: halve excitation; hold accepted gains fixed.\n');
                    continue;
                end
                stopReason="DIAGNOSED: "+string(diagnosis.Code);break;
            end
            baseline=false;
            if state.StagnantValidTrials>=cfg.StagnationTrials && search.CycleComplete && search.CompletedCycles>0 && ~search.CycleImproved
                stopReason="STAGNATION: 5 valid trials without improvement after a full joint cycle";break;
            end
        end
        if toc(clock)>=cfg.BudgetSec && ~startsWith(stopReason,"DIAGNOSED"),stopReason="WALL-TIME LIMIT";end
        summary=table(string(S.Name),attempts,validCount,state.BestScore,stopReason, ...
            'VariableNames',{'Stage','Trials','ValidTrials','BestScore','StopReason'});
        if isempty(Session.StageResults),Session.StageResults=summary;
        else,Session.StageResults=[Session.StageResults;summary];end
        qualityRow=table(string(S.Name),state.BestUsable,state.BestScore, ...
            'VariableNames',{'Stage','UsableOnSearchTest','Score'});
        if isempty(Session.Quality),Session.Quality=qualityRow;else,Session.Quality=[Session.Quality;qualityRow];end
        if startsWith(stopReason,"DIAGNOSED"),break;end
    end
    if height(Session.StageResults)==numel(Stages) && ~isempty(Session.StageResults)&&all(isfinite(Session.StageResults.BestScore)) && ...
            ~any(startsWith(Session.StageResults.StopReason,"DIAGNOSED"))
        Session.Status="COMPLETE";
    else
        Session.Status="DIAGNOSED FAILURE";
    end
catch ME
    Session.Status="ERROR";Session.FatalError=string(getReport(ME,'extended','hyperlinks','off'));
    fprintf(2,'%s\n',Session.FatalError);
end
Session.SearchConverged=Session.Status=="COMPLETE"&&cfg.RunMode=="tune"&& ...
    all(startsWith(Session.StageResults.StopReason,"STAGNATION"));
Session.MeetsInitialQualityScreen=Session.Status=="COMPLETE"&&height(Session.Quality)==numel(Stages)&&all(Session.Quality.UsableOnSearchTest);
Session.QualityNote="Response-quality screen on these tests, not physical-flight approval or independent robustness validation.";
Session.FinishedAt=datetime('now');
checkpoint();
if ~isempty(fieldnames(Session.BestGains)),assignin('base','PX4Tune',Session.BestGains);end
assignin('base','TuningSession',Session);
disp(Session.Results);disp(Session.StageResults);
fprintf('Session saved: %s\nReload: load_px4_tuning_session("%s")\n',Session.Folder,Session.Folder);
clear cleanup;
if options.ShowFigures,show_px4_tuning_session(Session);end

    function checkpoint()
        Session=save_px4_tuning_session(Session);
    end
end

function ensureInitialization(project)
required={'SimParams','SimscapeParams','CommParams','IC','PX4Tune','PX4Params','Config','Vehicle','TopPayloadVariant','Aero_On'};
ready=true;
for k=1:numel(required),ready=ready&&evalin('base',sprintf('exist(''%s'',''var'')==1',required{k}));end
if ~ready,initialize_px4_tuning_session(project);end
end
function T=emptyResults()
T=table('Size',[0 18],'VariableTypes',{'double','string','string','double','double','double','double', ...
    'double','double','double','double','double','double','string','string','logical','double','double'}, ...
    'VariableNames',{'Run','Stage','Parameter','Value','Amplitude','Score','BestScore','RMSE','Overshoot', ...
    'MinAltitude','MaxRate','RecoveryPeak','WallSeconds','Status','Diagnosis','Accepted','RelativeImprovement','StagnantValidTrials'});
T.ResultPrefix=strings(0,1);
T.SimulationTime=zeros(0,1);
T.StepCount=zeros(0,1);
end
function T=emptyHistory()
T=table('Size',[0 8],'VariableTypes',{'double','string','string','double','double','string','double','double'}, ...
    'VariableNames',{'Run','Stage','Parameter','TrialValue','AcceptedValue','Decision','Score','Amplitude'});
end
function T=historyRows(T,run,S,candidate,accepted,label,score)
for name=string([S.Parameters,S.FixedParameters])
    T=[T;table(run,string(S.Name),name,double(candidate.(name)),double(accepted.(name)), ...
        string(label),double(score),double(S.Amplitude),'VariableNames',T.Properties.VariableNames)]; %#ok<AGROW>
end
end
