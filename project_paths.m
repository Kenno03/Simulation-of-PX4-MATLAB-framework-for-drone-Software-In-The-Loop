function folders=project_paths(project)
%PROJECT_PATHS Add every project folder to the MATLAB path (single source of the folder list).
%   folders = project_paths(project) adds the project root and its code and asset folders:
%     parameters/          drone, motor, propeller and payload catalogues; parameter builders
%     px4/                 PX4 run helpers (startup dialog, stop, PX4_Receive setup, clock offset)
%     controller/          custom controller (LQR/LQI) parameters, control law and design
%     flight/              mission plans, trajectories, scoring and plots
%     flight/auto_tuning/  PID tuning logic, scoring and session files
%     tests/               offline tests
%     Drone Models/, Pictures/, Propeller/, step files/   models and their input assets
%   Called by initialization.m, the runners and the tests. Returns the folders it added.
if nargin<1||strlength(string(project))==0,project=fileparts(mfilename('fullpath'));end
sub=["","parameters","px4","controller","flight",fullfile("flight","auto_tuning"),"tests", ...
    "Drone Models","Pictures","Propeller","step files"];
folders=strings(1,0);
for s=sub
    f=fullfile(string(project),s);
    if isfolder(f),folders(end+1)=f;end %#ok<AGROW>
end
addpath(folders{:});
end
