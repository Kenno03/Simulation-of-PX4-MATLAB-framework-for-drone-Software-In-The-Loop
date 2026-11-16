function prepare_px4_tuning_workspace(cfg,S,tune)
%PREPARE_PX4_TUNING_WORKSPACE Restore build parameters for every trial.
% Initialization clears the base workspace. Never depend on an earlier mission.
project = fileparts(fileparts(fileparts(mfilename('fullpath'))));
assert(strcmpi(pwd,project),'MATLAB Current Folder must be %s.',project);
expected = {fullfile(project,'flight','flight_plan.m'), ...
    fullfile(project,'px4','configure_px4_receive_for_batch.m'), ...
    fullfile(project,'flight','auto_tuning','runCandidate.m')};
names = {'flight_plan','configure_px4_receive_for_batch','runCandidate'};
for k=1:numel(names)
    assert(strcmpi(which(names{k}),expected{k}), ...
        'Mixed checkout: %s resolves to %s; expected %s.', ...
        names{k},which(names{k}),expected{k});
end
models = {char(cfg.PX4Model),char(cfg.Plant)};
files = {fullfile(project,[models{1} '.slx']), ...
    fullfile(project,'Drone Models',[models{2} '.slx'])};
for k=1:numel(models)
    if bdIsLoaded(models{k})
        assert(strcmpi(get_param(models{k},'FileName'),files{k}), ...
            'Loaded model %s belongs to another checkout.',models{k});
        ws=get_param(models{k},'ModelWorkspace');
        assert(~ws.hasVariable('FlightPlan'), ...
            'Model workspace shadows base FlightPlan in %s; inspect before tuning.',models{k});
    end
end
assert(strcmp(string(tune.ID),"Default"),'The tuning pilot requires the Default profile.');
[~,~,duration]=px4_tuning_step_windows(S.Mode);
assert(cfg.ExpectedSteps==10 && cfg.SimTime>=cfg.TestStart+duration, ...
    'Each trial must allow ten steps and recovery after readiness.');
if ~isfield(cfg,'RunMode') || string(cfg.RunMode)=="verify"
    assert(cfg.StageLimit==1 && S.MaxTrials<=3, ...
        'Verification is one stage, at most three trials.');
end
plan=cfg.FlightPlan;
assert(plan.Size==0 && plan.Altitude==-double(cfg.HomeNED(3)), ...
    'Tuning background must hover at HomeNED altitude.');
assert(cfg.TestStart>=plan.TakeoffStart+plan.TakeoffTime+plan.SettleTime, ...
    'Excitation must start after takeoff and settling.');
assignin('base','FlightPlan',plan);
assignin('base','PX4Tune',tune);
assignin('base','TUNING_ON',true);
% PX4_Receive also contains the custom-controller (LQR) branch; it must compile but stays off while tuning.
assignin('base','CUSTOM_CONTROLLER',false);
assignin('base','CustomLQR',custom_controller_parameters());
assignin('base','TuningMode',uint8(S.Mode));
assignin('base','TuningStage',uint8(S.Mode));
assignin('base','TestAxis',uint8(S.Axis));
assignin('base','TestAmplitude',single(S.Amplitude));
assignin('base','HoverThrust',single(cfg.HoverThrust));
assignin('base','HomeNED',single(cfg.HomeNED));
assignin('base','TuningTestStart',double(cfg.TestStart));
assignin('base','TuningMinAltitude',double(cfg.MinTestAltitude));
assignin('base','TuningMaxClimbRate',double(cfg.MaxReadyClimbRate));
assignin('base','SIM_TIME',cfg.SimTime);
end
