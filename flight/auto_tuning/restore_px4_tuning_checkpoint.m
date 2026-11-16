function [Session,cfg,Stages,firstStage]=restore_px4_tuning_checkpoint(folder)
%RESTORE_PX4_TUNING_CHECKPOINT Reload a single-loop session that stopped on an execution fault.
%   [Session,cfg,Stages,firstStage] = restore_px4_tuning_checkpoint(folder) is used by
%   run_px4_tuning(ResumeFolder=folder). Only a diagnosed execution failure (e.g. a startup stall) can
%   be continued; the accepted gains, search state and history are kept.
% Resume an execution fault with accepted gains/search state and history intact.
[ok,info]=fileattrib(folder);assert(ok,'Session folder does not exist.');
folder=string(info.Name);
file=fullfile(folder,'session.mat');loaded=load(file,'Session');Session=loaded.Session;
assert(Session.Status=="DIAGNOSED FAILURE"&&Session.Results.Diagnosis(end)=="EXECUTION_FAILURE", ...
    'Automatic continuation is limited to a diagnosed execution failure.');
% A saved initialization can contain absolute CAD paths. Never restore it
% silently into another checkout, even if that checkout has the same names.
project=fileparts(fileparts(fileparts(mfilename('fullpath'))));
snapshotFile=fullfile(folder,Session.Results.ResultPrefix(end)+"CandidateSetup.mat");
assert(isfile(snapshotFile),'px4:MissingResumeProvenance', ...
    'Missing candidate setup; cannot establish the original source checkout.');
setup=load(snapshotFile,'snapshot');
assert(isfield(setup.snapshot,'CurrentFolder'),'px4:MissingResumeProvenance', ...
    'Candidate setup does not identify its source checkout.');
assert(strcmpi(setup.snapshot.CurrentFolder,project),'px4:ForeignResumeProject', ...
    'Resume this checkpoint from its recorded source folder: %s. Start a new session from another copy.', ...
    setup.snapshot.CurrentFolder);
assert(isfield(Session,'SearchVersion')&&Session.SearchVersion=="joint_fd_v1", ...
    'This folder resumes only joint-search checkpoints. Use the original folder for older sessions.');
Stages=Session.Stages;
firstStage=find([Stages.Key]==Session.CurrentStage);
assert(isscalar(firstStage)&&height(Session.StageResults)==firstStage);
assert(startsWith(Session.StageResults.StopReason(end),"DIAGNOSED: EXECUTION_FAILURE"));
assert(Session.StageResults.Stage(end)==Stages(firstStage).Name);
required={'StageState','SearchState','StageTrialCount','CurrentAmplitude','Initialization'};
assert(all(isfield(Session,required)),'Checkpoint lacks resumable optimizer state.');
cfg=Session.Config;
if ~isfield(Session,'OriginalBudgetSec'),Session.OriginalBudgetSec=cfg.BudgetSec;end
if ~isfield(Session,'WallDeadline')
    Session.WallDeadline=Session.StartedAt+seconds(Session.OriginalBudgetSec);
end
cfg.BudgetSec=seconds(Session.WallDeadline-datetime('now'));
assert(cfg.BudgetSec>0,'The original wall-time budget has expired.');
assert(Session.StageTrialCount<cfg.MaxTrials,'No trials remain in the interrupted stage.');
backup=fullfile(folder,"session_before_resume_"+string(datetime('now','Format','yyyyMMdd_HHmmss_SSS'))+".mat");
assert(~isfile(backup));copyfile(file,backup);
Session.StageResults(end,:)=[];
if height(Session.Quality)>=firstStage,Session.Quality(end,:)=[];end % Failure row stays in Results; replace only its unfinished stage summary.
Session.Folder=folder;cfg.OutputDirectory=char(folder);
Session.Status="RESUMING";Session.FinishedAt=NaT;Session.FatalError="";
if ~isfield(Session,'ResumeHistory'),Session.ResumeHistory=strings(0,1);end
Session.ResumeHistory(end+1)=string(datetime('now'))+" | execution failure after trial "+height(Session.Results);
end
