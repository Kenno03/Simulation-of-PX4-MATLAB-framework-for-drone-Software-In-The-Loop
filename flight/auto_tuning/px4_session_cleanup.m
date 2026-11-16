function px4_session_cleanup(folder,models)
%PX4_SESSION_CLEANUP Stop the run and close a tuning session cleanly (onCleanup callback).
%   px4_session_cleanup(folder,{PX4Model,Plant}) stops a running simulation, marks an unfinished
%   session.mat as INTERRUPTED, switches the PX4_Receive scopes back on (they are off during
%   automated runs, see configure_px4_receive_for_batch), resets the tuning switches in the base
%   workspace and closes the logs. Called with values, not a nested workspace, so it also works when
%   the runner exits with an error.
for model=models
    if bdIsLoaded(model{1})&&~strcmp(get_param(model{1},'SimulationStatus'),'stopped')
        try,stop_px4_run();catch ME,fprintf(2,'Cleanup: %s\n',ME.message);end
        break;
    end
end
if bdIsLoaded(models{1})
    try
        scopes=find_system(models{1},'LookUnderMasks','all','FollowLinks','on','BlockType','Scope');
        for k=1:numel(scopes),set_param(scopes{k},'Commented','off');end
    catch ME
        fprintf(2,'Cleanup (scopes): %s\n',ME.message);
    end
end
filename=fullfile(folder,'session.mat');
if isfile(filename)
    try
        data=load(filename,'Session');
        if ismember(data.Session.Status,["INITIALIZING","RUNNING"])
            data.Session.Status="INTERRUPTED";data.Session.FinishedAt=datetime('now');
            save_px4_tuning_session(data.Session);
        end
    catch ME
        fprintf(2,'Interrupted-session checkpoint: %s\n',ME.message);
    end
end
assignin('base','TUNING_ON',false);assignin('base','TuningMode',uint8(0));
try,sldiagviewer.diary('off');catch,end
diary off;
end
