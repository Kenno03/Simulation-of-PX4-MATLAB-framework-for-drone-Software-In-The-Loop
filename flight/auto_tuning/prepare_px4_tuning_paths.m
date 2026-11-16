function project=prepare_px4_tuning_paths()
%PREPARE_PX4_TUNING_PATHS Select this checkout and reject loaded foreign models.
project=fileparts(fileparts(fileparts(mfilename('fullpath'))));
modelFiles={fullfile(project,'PX4_Receive.slx'), ...
    fullfile(project,'Drone Models','Drone_simulation_PX4.slx'), ...
    fullfile(project,'Drone Models','Simscape_Drone_Model.slx')};
for k=1:numel(modelFiles)
    [~,name]=fileparts(modelFiles{k});
    if bdIsLoaded(name)
        assert(strcmpi(get_param(name,'FileName'),modelFiles{k}), ...
            'Close the model from another checkout before continuing: %s.', ...
            get_param(name,'FileName'));
    end
end
% Do not silently remove user paths: reject folders of another copy of this project (they contain
% the project's own functions but lie outside this checkout).
entries=strsplit(path,pathsep);
for k=1:numel(entries)
    e=entries{k};if isempty(e),continue,end
    inProject=strcmpi(e,project)||startsWith(lower(e),[lower(project) filesep]);
    projectFiles=isfile(fullfile(e,'project_paths.m'))||isfile(fullfile(e,'run_mission.m'))|| ...
        isfile(fullfile(e,'px4_tuning_parameters.m'))||isfile(fullfile(e,'flight_plan.m'));
    assert(inProject||~projectFiles,'Remove the other checkout from the MATLAB path: %s.',e);
end
cd(project);
addpath(project);project_paths(project);
clear configure_px4_receive_for_batch runCandidate;
rehash;
fprintf('Current Folder: %s\n',pwd);
which flight_plan -all
which configure_px4_receive_for_batch -all
which runCandidate -all
end
