function Session=run_px4_guide_tuning(groups,options)
%RUN_PX4_GUIDE_TUNING Automatic PX4 PID tuning, one loop group at a time, multi-axis flights.
% Each flight excites every axis of a group in sequence (rates: roll/pitch/yaw; attitude:
% roll/pitch/yaw; velocity and position: horizontal/vertical), so one takeoff serves all axes.
% Per group: baseline repeats (start score and noise margin) -> search steps per axis in parallel
% -> next group. Order: rates -> attitude -> velocity -> position. A final check flies every group.
%
% Two search methods (Method=...):
%   "guide"  rules of the PX4 tuning guide (px4_guide_axis): raise gains until oscillation or
%            overshoot, back off.
%   "rmse"   pattern search that minimises the RMSE step-response score per axis (px4_rmse_axis):
%            J = RMSE/A + 0.30*max|e|/A + 0.25*overshoot. A change must beat the best score by more
%            than the repeat noise. After each group a robustness probe flies every loop gain x1.3;
%            an axis that oscillates or goes unstable is backed off x0.85 (gain margin >= 1.3).
%
%   TuningSession = run_px4_guide_tuning();                       % guide tuning from Default
%   TuningSession = run_px4_guide_tuning("all",Method="rmse");    % RMSE tuning from Default
%   TuningSession = run_px4_guide_tuning("rates");                % one group
%   TuningSession = run_px4_guide_tuning("all",Mode="validate");  % fly current PX4Tune, no tuning
arguments
    groups (1,:) string = "all"
    options.Method (1,1) string {mustBeMember(options.Method,["guide","rmse"])} = "guide"
    options.Mode (1,1) string {mustBeMember(options.Mode,["tune","validate"])} = "tune"
    options.MaxHours (1,1) double {mustBePositive} = 24
    options.Repeats (1,1) double {mustBeInteger,mustBePositive} = 3
    options.MaxFlightsPerGroup (1,1) double {mustBeInteger,mustBePositive} = 20
    options.FinalCheck (1,1) logical = true
    options.ShowFigures (1,1) logical = true
    options.OutputRoot (1,1) string = ""
    options.FlightFunction = []   % tests only: replaces runCandidate; no model is loaded or flown
end
project=fileparts(mfilename('fullpath'));
dryRun=~isempty(options.FlightFunction);
if dryRun,flightFunction=options.FlightFunction;else,flightFunction=@runCandidate;end
if options.Method=="rmse",axisLogic=@px4_rmse_axis;else,axisLogic=@px4_guide_axis;end
addpath(project);project_paths(project);
prepare_px4_tuning_paths();
[cfg,Groups]=px4_guide_configuration(groups,MaxHours=options.MaxHours,Repeats=options.Repeats, ...
    MaxFlightsPerGroup=options.MaxFlightsPerGroup);
px4_session_lock("acquire");lockGuard=onCleanup(@()px4_session_lock("release")); %#ok<NASGU>
if options.OutputRoot=="",root=fullfile(project,'TuningSessions');else,root=options.OutputRoot;end
if ~isfolder(root),mkdir(root);end
folder=fullfile(root,"PX4_"+string(datetime('now','Format','yyyyMMdd_HHmmss'))+"_"+options.Method+"_"+options.Mode);
assert(~isfolder(folder),'Session folder exists.');mkdir(folder);cfg.OutputDirectory=char(folder);
cfg.Method=options.Method;
Session=struct('Version',"guide_v1",'Method',options.Method,'Folder',string(folder),'Mode',options.Mode,'StartedAt',datetime('now'), ...
    'FinishedAt',NaT,'Status',"INITIALIZING",'Config',cfg,'Groups',Groups,'Flights',table(), ...
    'Decisions',strings(0,1),'StartGains',struct(),'BestGains',struct(),'Final',table(), ...
    'Initialization',struct(),'MeetsQualityScreen',false,'FatalError',"");
diary(fullfile(folder,'console_log.txt'));
cleanup=onCleanup(@()px4_session_cleanup(char(folder),{cfg.PX4Model,cfg.Plant}));
clock=tic;run=0;
try
    required={'SimParams','SimscapeParams','CommParams','IC','PX4Tune','PX4Params','Config','Vehicle','TopPayloadVariant','Aero_On'};
    ready=true;for k=1:numel(required),ready=ready&&evalin('base',sprintf('exist(''%s'',''var'')==1',required{k}));end
    if ~ready,initialize_px4_tuning_session(project);end
    for k=1:numel(required),Session.Initialization.(required{k})=evalin('base',required{k});end
    tune=Session.Initialization.PX4Tune;
    assert(strcmp(string(tune.ID),"Default"),'Select Default in Main before tuning.');
    defaults=px4_tuning_parameters("Default");
    for name=px4_controller_gain_names(),if ~isfield(tune,name),tune.(name)=defaults.(name);end,end
    if options.Mode=="tune",tune=px4_default_seed(tune,defaults);end
    Session.StartGains=tune;Session.BestGains=tune;
    if ~dryRun
        prepare_px4_tuning_workspace(cfg,Groups(1),tune);
        load_system(fullfile(project,'PX4_Receive.slx'));
        load_system(fullfile(project,'Drone Models','Drone_simulation_PX4.slx'));
        load_system(fullfile(project,'Drone Models','Simscape_Drone_Model.slx'));
        if ~strcmp(get_param(cfg.Plant,'SimulationStatus'),'stopped')||~strcmp(get_param(cfg.PX4Model,'SimulationStatus'),'stopped')
            stop_px4_run();
        end
        configure_px4_receive_for_batch(cfg.PX4Model);
    end
    Session.Status="RUNNING";checkpoint();
    fprintf('Tuning session (%s): %s | mode %s | groups %s | budget %.1f h\n',options.Method,folder,options.Mode,strjoin([Groups.Key],','),options.MaxHours);
    if options.Mode=="validate"
        for G=Groups,[~,ok]=fly(G,tune,"validate");if ~ok,error('Execution failure in validation of %s.',G.Key);end,end
    else
        for G=Groups
            tune=tuneGroup(G,tune);Session.BestGains=tune;checkpoint();
        end
        if options.FinalCheck
            for G=Groups
                [mets,ok]=fly(G,tune,"final check");if ~ok,error('Execution failure in final check of %s.',G.Key);end
                for j=1:numel(G.Members)
                    Session.Final=[Session.Final;table(G.Key,G.Members(j).Key,mets(j).Valid,mets(j).Score,mets(j).Tracking, ...
                        mets(j).LateError,mets(j).OvershootRMS,mets(j).Oscillation,mets(j).MotorVariation,mets(j).Overshoot,mets(j).Usable, ...
                        'VariableNames',{'Group','Loop','Valid','Score','Tracking','LateError','OvershootRMS','Oscillation','MotorVariation','OvershootPercent','QualityScreen'})];
                end
            end
            Session.MeetsQualityScreen=~isempty(Session.Final)&&all(Session.Final.Valid&Session.Final.QualityScreen);
        end
    end
    Session.Status="COMPLETE";
catch ME
    Session.Status="STOPPED";Session.FatalError=string(getReport(ME,'extended','hyperlinks','off'));
    fprintf(2,'%s\n',Session.FatalError);
end
Session.FinishedAt=datetime('now');checkpoint();
assignin('base','PX4Tune',Session.BestGains);assignin('base','TuningSession',Session);
if ~isempty(Session.Final),disp(Session.Final);end
fprintf('Session saved: %s\nStatus: %s | flights: %d | wall %.2f h\n',folder,Session.Status,run,toc(clock)/3600);
clear cleanup;
if options.ShowFigures&&isfile(fullfile(folder,'GuideGainHistory.fig')),openfig(fullfile(folder,'GuideGainHistory.fig'),'visible');end

    %% ------------------------------------------------------------------
    function tune=tuneGroup(G,tune)
        A=struct([]);
        for j=1:numel(G.Members)
            a=axisLogic("init",[],G,j,tune);if isempty(A),A=a;else,A(j)=a;end
        end
        flightsInGroup=0;base=cell(1,numel(G.Members));
        for r=1:cfg.Repeats
            [mets,ok]=fly(G,tune,sprintf("baseline %d/%d",r,cfg.Repeats));flightsInGroup=flightsInGroup+1;
            if ~ok,error('Execution failure during %s baseline.',G.Key);end
            for j=1:numel(A),base{j}=[base{j} mets(j)];end
        end
        for j=1:numel(A),A(j)=axisLogic("baseline",A(j),base{j});note(G.Members(j).Name+": "+A(j).Log(end));end
        while true
            candidate=tune;notes=strings(1,numel(A));needFlight=false;
            for j=1:numel(A)
                [A(j),g,notes(j)]=axisLogic("propose",A(j));
                for n=string(fieldnames(g))',candidate.(n)=g.(n);end
                needFlight=needFlight||A(j).Pending.Kind~="hold";
            end
            if ~needFlight,break;end
            if flightsInGroup>=cfg.MaxTrials,note(G.Key+": flight limit reached; current accepted gains kept.");break;end
            if toc(clock)>=cfg.BudgetSec,note(G.Key+": wall-time budget reached.");break;end
            [mets,ok]=fly(G,candidate,strjoin(notes," | "));flightsInGroup=flightsInGroup+1;
            if ~ok,error('Execution failure during %s tuning.',G.Key);end
            for j=1:numel(A)
                [A(j),g,why]=axisLogic("observe",A(j),mets(j));
                for n=string(fieldnames(g))',tune.(n)=g.(n);end
                if why~=""&&why~="continue"&&why~="measured",note(G.Members(j).Name+": "+why);end
            end
            Session.BestGains=tune;checkpoint();
        end
        if options.Method=="rmse",[A,tune]=robustnessProbe(G,A,tune,flightsInGroup);end
        for j=1:numel(A),for k=1:numel(A(j).Log),note(G.Members(j).Name+" | "+A(j).Log(k));end,end
        Session.GroupAxes.(G.Key)=A;
    end

    function [A,tune]=robustnessProbe(G,A,tune,flightsInGroup)
        % Fly every loop gain of the group x1.3. An axis that oscillates or goes unstable is backed
        % off x0.85 and the probe is repeated (at most 3 probes, within the group's flight limit).
        for attempt=1:3
            if flightsInGroup>=cfg.MaxTrials||toc(clock)>=cfg.BudgetSec
                note(G.Key+": robustness probe skipped (flight or time limit).");return;
            end
            probe=tune;
            for j=1:numel(A),[~,g]=axisLogic("probe",A(j),1.3);for n=string(fieldnames(g))',probe.(n)=g.(n);end,end
            [mets,ok]=fly(G,probe,"robustness probe x1.3");flightsInGroup=flightsInGroup+1;
            if ~ok,error('Execution failure during %s robustness probe.',G.Key);end
            failed=false;
            for j=1:numel(A)
                if ~mets(j).Valid||mets(j).Unstable||mets(j).Oscillation>0.15
                    failed=true;[A(j),g]=axisLogic("backoff",A(j),0.85);
                    for n=string(fieldnames(g))',tune.(n)=g.(n);end
                    note(G.Members(j).Name+": robustness probe x1.3 failed; loop gain x0.85.");
                end
            end
            Session.BestGains=tune;checkpoint();
            if ~failed,note(G.Key+": robustness probe x1.3 passed.");return;end
        end
        note(G.Key+": robustness probe still failing after 3 back-offs; last gains kept.");
    end

    function [mets,ok]=fly(G,tuneToFly,purpose)
        ok=false;mets=struct([]);
        % Every gain flown is a value PX4 accepts: rounded to its PX4 resolution (the axis logic
        % already proposes rounded gains; this also covers the start gains).
        for gainName=px4_controller_gain_names()
            [~,~,res]=px4_controller_gain_limits(gainName);
            tuneToFly.(gainName)=round(tuneToFly.(gainName),round(-log10(res)));
        end
        for attempt=1:2
            run=run+1;cfgG=cfg;cfgG.SimTime=G.SimTime;cfgG.TrialPrefix=sprintf('Flight%03d_%s_',run,G.Key);
            fprintf('\n[%s] flight %d (%s) | %s\n',string(datetime('now','Format','HH:mm')),run,G.Key,purpose);
            [M,status,wall,diagnosis]=flightFunction(cfgG,G,tuneToFly,clock);
            if ~iscell(M)
                % Execution failure before scoring (e.g. startup stall): retry the same flight once.
                record(G,tuneToFly,purpose,attempt,[],status,wall,diagnosis);
                note(sprintf('Flight %d execution failure (%s).',run,status));
                if attempt==1,continue;else,return;end
            end
            for j=1:numel(M),m=metrics(M{j},status(j),diagnosis{j});if isempty(mets),mets=m;else,mets(j)=m;end,end
            record(G,tuneToFly,purpose,attempt,mets,status,wall,diagnosis);ok=true;checkpoint();return;
        end
    end

    function record(G,tuneToFly,purpose,attempt,mets,status,wall,diagnosis)
        for j=1:numel(G.Members)
            mem=G.Members(j);names=string(struct2cell(mem.Gains))';
            gains=strjoin(arrayfun(@(n)sprintf('%s=%.6g',n,tuneToFly.(n)),names,'UniformOutput',false),' ');
            gains=string(gains);
            if isempty(mets)
                row=table(run,attempt,G.Key,mem.Key,string(purpose),gains,string(status(1)),"EXECUTION_FAILURE", ...
                    NaN,NaN,NaN,NaN,NaN,NaN,NaN,false,wall);
            else
                d=diagnosis{j};x=mets(j);
                row=table(run,attempt,G.Key,mem.Key,string(purpose),gains,string(status(j)),string(d.Code), ...
                    x.Score,x.Tracking,x.LateError,x.OvershootRMS,x.Oscillation,x.MotorVariation,x.Overshoot,x.Usable,wall);
            end
            row.Properties.VariableNames={'Flight','Attempt','Group','Loop','Purpose','Gains','Status','Diagnosis', ...
                'Score','Tracking','LateError','OvershootRMS','Oscillation','MotorVariation','OvershootPercent','QualityScreen','WallSeconds'};
            row.RMSEScore=NaN;if ~isempty(mets),row.RMSEScore=mets(j).RMSEScore;end
            Session.Flights=[Session.Flights;row];
        end
    end

    function note(text)
        Session.Decisions(end+1,1)=string(text);fprintf('  %s\n',text);
    end

    function checkpoint()
        Session=px4_guide_save(Session);
    end
end

function m=metrics(M,status,d)
m=struct('Valid',status=="VALID",'Unstable',any(string(d.Code)==["EXCITATION_OR_RECOVERY_UNSTABLE","BASELINE_UNSTABLE"]), ...
    'Overshoot',NaN,'Tracking',NaN,'LateError',NaN,'OvershootRMS',NaN,'Oscillation',NaN,'MotorVariation',NaN, ...
    'Score',NaN,'RMSEScore',NaN,'Usable',false);
if contains(status,["safety limit","saturation"]),m.Unstable=true;end
% RMSE step-response score (score_px4_tuning_candidate keeps it as LegacyTrackingScore)
if m.Valid&&isfield(M,'LegacyTrackingScore'),m.RMSEScore=M.LegacyTrackingScore;end
if m.Valid&&isfield(M,'JointFeatures')
    F=M.JointFeatures;
    for f=["Tracking","LateError","OvershootRMS","Oscillation","MotorVariation","Score"],m.(f)=F.(f);end
    m.Usable=logical(F.Usable);
    if isfield(d,'Response'),m.Overshoot=d.Response.OvershootPercent;end
end
if m.Valid&&~isfinite(m.Overshoot),m.Valid=false;end
end
