function Result=run_mission(plan,options)
%RUN_MISSION Fly a mission (square, circle or figure-8) in PX4 SITL, score it and save plots.
%   Result = run_mission(plan) flies FlightPlan PLAN once with the PX4 gains in the base workspace
%   (PX4Tune) and scores it on the plant truth (score_mission).
%   Result = run_mission(plan,Gains="HandTuned_RealDrone",Repeats=3) flies a stored gain profile 3 times.
%   Result = run_mission(plan,Gains=TuningSession.BestGains,Name="RMSE-tuned") flies tuned gains.
%   Result = run_mission(plan,Controller="lqi") flies the custom controller (LQR + integral action;
%   "lqr" for the plain LQR) instead of PX4's PID loops.
%
%   Example (figure-8 with feedforward and heading along the path):
%       plan = flight_plan("figure8"); plan.Size = 4; plan.Altitude = 5;
%       plan.SettleTime = 10; plan.LegTime = 5; plan.DwellTime = 0; plan.FinalHoverTime = 18;
%       plan.VelocityFeedforward = true; plan.HeadingAlongPath = true;
%       A = run_mission(plan,Gains="Default",Repeats=3);
%       B = run_mission(plan,Gains="HandTuned_RealDrone",Repeats=3);
%       compare_missions([A B]);
%
%   Options
%     Gains       profile name for px4_tuning_parameters, a gain struct, or "" (base PX4Tune). Gains
%                 missing from a struct are taken from Default.
%     Controller  "px4", "lqi" or "lqr"; default: "lqi" for the "LQI" profile, otherwise "px4".
%     Repeats     number of flights (1).
%     Name        label used in tables and plots (default: profile or controller name).
%     OutputRoot  folder for the results (default <project>/FlightValidation_<time>, git-ignored).
%     Plots       true: detailed plots per flight (export_flight_plots).
%     ShowScopes  false: PX4_Receive scopes off while flying (about twice as fast; nothing is lost).
%   Result: Name, Plan, Gains, Controller, Flights (one row per flight: status, wall time, scores),
%   Summary (mean and std of each score), Scores (score_mission structs) and Folder.
%   The base workspace must be initialised (Main.mlx); otherwise the default setup is built.
%   An execution failure (for example the known 16 s startup stall) is retried once.
arguments
    plan (1,1) struct
    options.Gains = ""
    options.Controller (1,1) string {mustBeMember(options.Controller,["","px4","lqi","lqr"])} = ""
    options.Repeats (1,1) double {mustBeInteger,mustBePositive} = 1
    options.Name (1,1) string = ""
    options.OutputRoot (1,1) string = ""
    options.Plots (1,1) logical = true
    options.ShowScopes (1,1) logical = false
end
project=fileparts(mfilename('fullpath'));
addpath(project);project_paths(project);
prepare_px4_tuning_paths();
px4_session_lock("acquire");lockGuard=onCleanup(@()px4_session_lock("release")); %#ok<NASGU>
required={'SimParams','SimscapeParams','CommParams','IC','PX4Tune','PX4Params','Config','Vehicle','TopPayloadVariant','Aero_On'};
ready=true;for k=1:numel(required),ready=ready&&evalin('base',sprintf('exist(''%s'',''var'')==1',required{k}));end
if ~ready,initialize_px4_tuning_session(project);end
% gains and name
def=px4_tuning_parameters("Default");
if isstruct(options.Gains),tune=options.Gains;source="struct";
elseif strlength(string(options.Gains))>0,tune=px4_tuning_parameters(string(options.Gains));source=string(options.Gains);
else,tune=evalin('base','PX4Tune');source="base PX4Tune";
end
for n=string(fieldnames(def))',if ~isfield(tune,n),tune.(n)=def.(n);end,end
if options.Controller==""   % follow the profile: "LQI" selects the custom controller
    if tune.CustomController,options.Controller="lqi";else,options.Controller="px4";end
end
name=options.Name;
if name==""
    if options.Controller~="px4",name=upper(options.Controller);elseif isfield(tune,'ID'),name=string(tune.ID);else,name=source;end
end
custom=options.Controller~="px4";
if custom,[CustomLQR,lqiSetup]=custom_controller_parameters(options.Controller);else,[CustomLQR,lqiSetup]=custom_controller_parameters();end
checkTune=tune;checkTune.CustomController=double(custom);           % warn if not validated on this setup
check_gain_validity(checkTune,evalin('base','SimParams'),lqiSetup);
simTime=plan.TakeoffStart+plan.TakeoffTime+plan.SettleTime+4*(plan.LegTime+plan.DwellTime)+plan.FinalHoverTime;
root=options.OutputRoot;if root=="",root=fullfile(project,"FlightValidation_"+string(datetime('now','Format','yyyyMMdd_HHmmss')));end
folder=fullfile(root,matlab.lang.makeValidName(char(name)));if ~isfolder(folder),mkdir(folder);end
diary(fullfile(folder,'console_log.txt'));diaryGuard=onCleanup(@()diary('off')); %#ok<NASGU>
% models
model='PX4_Receive';plant='Drone_simulation_PX4';cfg=px4_guide_configuration("rates");
load_system(fullfile(project,[model '.slx']));
load_system(fullfile(project,'Drone Models',[plant '.slx']));
load_system(fullfile(project,'Drone Models','Simscape_Drone_Model.slx'));
set_param('Simscape_Drone_Model','SimMechanicsOpenEditorOnUpdate','off');
cleanup=onCleanup(@()px4_session_cleanup(char(folder),{model,plant}));
if ~strcmp(get_param(plant,'SimulationStatus'),'stopped')||~strcmp(get_param(model,'SimulationStatus'),'stopped'),stop_px4_run();end
configure_px4_receive_for_batch(model,ShowScopes=options.ShowScopes);
set_param(plant,'StopTime','inf','SignalLogging','on');
set_param(model,'StopTime',num2str(simTime),'ReturnWorkspaceOutputs','off');
% workspace of the flight (mission branch on, tuning branch off)
assignin('base','FlightPlan',plan);assignin('base','PX4Tune',tune);
assignin('base','TUNING_ON',false);assignin('base','TuningMode',uint8(0));assignin('base','TuningStage',uint8(0));
assignin('base','TestAxis',uint8(1));assignin('base','TestAmplitude',single(0));
assignin('base','HoverThrust',single(cfg.HoverThrust));assignin('base','HomeNED',single(cfg.HomeNED));
assignin('base','TuningTestStart',double(cfg.TestStart));assignin('base','TuningMinAltitude',double(cfg.MinTestAltitude));
assignin('base','TuningMaxClimbRate',double(cfg.MaxReadyClimbRate));assignin('base','SIM_TIME',simTime);
assignin('base','CUSTOM_CONTROLLER',custom);assignin('base','CustomLQR',CustomLQR);
save(fullfile(folder,'setup.mat'),'plan','tune','CustomLQR','options','simTime');
rows=cell(options.Repeats,1);Scores=cell(options.Repeats,1);
for r=1:options.Repeats
    fprintf('\n===== %s: flight %d/%d (%s) =====\n',name,r,options.Repeats,string(datetime('now','Format','HH:mm:ss')));
    flightFolder=fullfile(folder,sprintf('flight%02d',r));if ~isfolder(flightFolder),mkdir(flightFolder);end
    for attempt=1:2
        prior=Simulink.sdi.getAllRunIDs;t0=tic;status="OK";watcher=[];
        try
            watcher=px4_ack_popup(1800);
            set_param(plant,'SimulationCommand','start');waitForPlant(plant,120);
            set_param(model,'SimulationCommand','start');waitForPX4(model,simTime,2400);
            stop_px4_run(watcher);watcher=[];
        catch ME
            status="ERROR: "+string(ME.message);fprintf(2,'attempt %d: %s\n',attempt,status);
            try,stop_px4_run(watcher);catch,end
        end
        if status=="OK",break,end
    end
    wall=toc(t0);
    ids=setdiff(Simulink.sdi.getAllRunIDs,prior);plantData=[];ctrlData=[];
    for k=1:numel(ids)
        run=Simulink.sdi.getRun(ids(k));d=Simulink.sdi.exportRun(ids(k));n=string(run.Name);
        save(fullfile(flightFolder,sprintf('%s_sdi.mat',matlab.lang.makeValidName(char(n)))),'d','-v7.3');
        if contains(n,plant),plantData=d;end
        if contains(n,model),ctrlData=d;end
    end
    row=table(r,status,wall/60,'VariableNames',{'Flight','Status','WallMin'});
    if status=="OK"
        try
            if ~custom
                trig=ctrlData.getElement('PX4_PARAMETER_WRITE_TRIGGER').Values;
                assert(any(trig.Data~=0,'all'),'Parameter-write trigger never fired: gains not applied.');
            end
            offset=controller_clock_offset(ctrlData);M=score_mission(plantData,plan,offset,simTime);
            save(fullfile(flightFolder,'score.mat'),'M','offset');Scores{r}=M;row=[row,M.Summary]; %#ok<AGROW>
            if options.Plots,export_flight_plots(plantData,fullfile(flightFolder,'plots'),plan,offset);end
        catch ME
            row.Status="INVALID: "+string(ME.message);
        end
    end
    rows{r}=row;
    fprintf('%s flight %d: %s (%.1f min)\n',name,r,row.Status,wall/60);
end
Flights=stackRows(rows);Summary=summarise(Flights);
Result=struct('Name',name,'Plan',plan,'Gains',tune,'Controller',options.Controller,'Flights',Flights, ...
    'Summary',Summary,'Scores',{Scores},'Folder',string(folder));
save(fullfile(folder,'result.mat'),'Result');writetable(Flights,fullfile(folder,'flights.csv'));
if ~isempty(Summary),disp(Summary);end
end

function S=summarise(F)
S=table();ok=startsWith(F.Status,"OK");if ~any(ok)||width(F)<4,return;end
for v=string(F.Properties.VariableNames(4:end))
    x=F.(v)(ok);S.(v+"_mean")=mean(x);S.(v+"_std")=std(x);
end
S.Flights=nnz(ok);
end
function F=stackRows(rows)
% Failed flights have no score columns: pad every row to the union of the columns, then stack.
names=strings(1,0);
for k=1:numel(rows),names=[names setdiff(string(rows{k}.Properties.VariableNames),names,'stable')];end %#ok<AGROW>
F=table();
for k=1:numel(rows)
    row=rows{k};
    for v=names,if ~ismember(v,row.Properties.VariableNames),row.(v)=NaN;end,end
    F=[F;row(:,names)]; %#ok<AGROW>
end
end
function waitForPlant(plant,timeout)
t=tic;
while ~strcmp(get_param(plant,'SimulationStatus'),'running')
    assert(toc(t)<timeout,'Plant did not start within %.0f s.',timeout);pause(0.5);
end
end
function waitForPX4(model,simTime,timeout)
t=tic;seen=false;last=-inf;lastT=-inf;lastAdv=0;
while true
    assert(toc(t)<timeout,'PX4 run exceeded %.0f s wall time.',timeout);
    connected=strcmp(get_param(model,'ExtModeConnected'),'on');state=string(get_param(model,'SimulationStatus'));
    simNow=str2double(string(get_param(model,'SimulationTime')));seen=seen||connected||simNow>0;
    if isfinite(simNow)&&simNow>lastT+1e-6,lastT=simNow;lastAdv=toc(t);end
    assert(~seen||toc(t)-lastAdv<120,'PX4 model time stalled at %.3f s.',simNow);
    if simNow-last>=10,fprintf('PX4 simulation: %.1f / %.0f s\n',simNow,simTime);last=simNow;end
    if ~seen&&state=="stopped"&&toc(t)<120,pause(1);continue;end
    assert(state~="stopped"||simNow>=simTime-0.05,'PX4 stopped early at %.3f s.',simNow);
    if simNow>=simTime-0.05,return;end
    assert(connected||state=="running"||state=="external",'PX4 external-mode connection lost.');
    pause(1);
end
end
