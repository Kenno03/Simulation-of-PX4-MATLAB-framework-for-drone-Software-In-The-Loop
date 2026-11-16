function [figures,files]=show_px4_tuning_session(Session,options)
%SHOW_PX4_TUNING_SESSION Open the saved figures of a single-loop tuning session (run_px4_tuning).
%   show_px4_tuning_session(Session)                 summary figures of the session
%   show_px4_tuning_session(Session,Trial=12)        figures saved for trial (Run) 12
%   show_px4_tuning_session(Session,AllFigures=true) every saved figure
%   [figures,files] = show_px4_tuning_session(...) also returns the figure handles and FIG files.
%   Session is the struct returned by run_px4_tuning or load_px4_tuning_session.
arguments
    Session (1,1) struct
    options.Trial (1,1) double {mustBeInteger,mustBeNonnegative} = 0
    options.AllFigures (1,1) logical = false
end
assert(~(options.Trial>0&&options.AllFigures),'Choose a trial or all figures.');
names=string(Session.FigureFiles(:));
if options.Trial>0
    assert(ismember('ResultPrefix',Session.Results.Properties.VariableNames), ...
        'This archive does not contain per-trial filenames.');
    row=Session.Results.Run==options.Trial;
    assert(nnz(row)==1,'Choose a Run number from the session results table.');
    names=names(startsWith(names,Session.Results.ResultPrefix(row)));
elseif ~options.AllFigures
    names=names(ismember(names,["GainHistory.fig","ScoreHistory.fig"]));
end
files=fullfile(Session.Folder,names);
files=files(isfile(files));
figures=gobjects(0,1);
if ~usejava('desktop'),return;end
for filename=files'
    figures(end+1,1)=openfig(filename,'new','visible'); %#ok<AGROW>
end
drawnow;
if options.Trial==0&&~options.AllFigures
    fprintf('Inspect a saved trial with show_px4_tuning_session(TuningSession,Trial=n), using its Run number.\n');
end
end
