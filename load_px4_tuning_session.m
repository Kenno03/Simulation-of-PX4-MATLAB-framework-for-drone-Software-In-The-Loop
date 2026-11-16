function Session=load_px4_tuning_session(folder,options)
%LOAD_PX4_TUNING_SESSION Reload tables, settings and saved MATLAB figures.
arguments
    folder (1,1) string
    options.ShowFigures (1,1) logical = true
end
if isfile(folder),[folder,~,~]=fileparts(folder);end
[ok,info]=fileattrib(folder);assert(ok,'Session folder does not exist.');folder=string(info.Name);
data=load(fullfile(folder,'session.mat'),'Session');Session=data.Session;
% File references within the session are relative, so copied sessions reload.
Session.Folder=string(folder);
disp(Session.Results);disp(Session.StageResults);
assignin('base','TuningSession',Session);
if options.ShowFigures,show_px4_tuning_session(Session);end
end
