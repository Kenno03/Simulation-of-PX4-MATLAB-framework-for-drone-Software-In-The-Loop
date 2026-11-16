function test_px4_matlab_session()
%TEST_PX4_MATLAB_SESSION Offline checks of single-loop session handling (no simulation).
%   test_px4_matlab_session() checks the menu-to-stage mapping, the session configuration, the
%   acceptance and convergence rules, and that saved session archives reload after relocation.
% Tests session selection, convergence, MATLAB archives and relocation.
options=struct('Mode',"verify",'MaxTrials',3,'ShowFigures',false,'MinimumImprovement',0.01);
[cfg,S]=px4_session_configuration(px4_tuning_selection(3),options);
assert(isscalar(S)&&S.Key=="rollrate"&&S.Axis==1&&S.Mode==1&&cfg.MaxTrials==3);
assert(isequal(px4_tuning_selection(2),["rollrate","pitchrate","yawrate"]));
options.Mode="tune";options.MaxTrials=30;
[~,allStages]=px4_session_configuration(px4_tuning_selection(1),options);
assert(numel(allStages)==10 && ~any(contains(lower([allStages.Name]),'filter')));
assert(isequal(sort(unique(string([allStages.Parameters,allStages.FixedParameters]))),sort(px4_controller_gain_names())));
assert(px4_tuning_selection(16)=="yawattitude");
assert(~any([allStages.Mode]==5),'No acceleration stage may be implied.');
state=struct('BestScore',inf,'BestTune',struct('Gain',0.003), ...
    'StagnantValidTrials',0,'HasBaseline',false);
[state,accepted]=px4_tuning_accept_trial(state,struct('Gain',0.003),100,true,.01);
assert(accepted&&state.StagnantValidTrials==0);
for k=1:5
    [state,accepted]=px4_tuning_accept_trial(state,struct('Gain',0.003*(1+.05*k)),99.5,true,.01);
    assert(~accepted&&state.StagnantValidTrials==k);
end
[state,~]=px4_tuning_accept_trial(state,struct('Gain',0.001),inf,false,.01);
assert(state.StagnantValidTrials==5,'Failed trials do not advance stagnation.');
[state,accepted]=px4_tuning_accept_trial(state,struct('Gain',0.00301),90,true,.01);
assert(accepted&&state.StagnantValidTrials==0,'Meaningful response improvement resets the counter, even for a tiny gain delta.');
px4_session_lock("acquire");rejected=false;
try,px4_session_lock("acquire");catch,rejected=true;end
px4_session_lock("release");assert(rejected,'Concurrent MATLAB session ownership must be rejected.');
test_px4_tuning_logic();
folder=tempname;mkdir(folder);
Session=struct('Folder',string(folder),'Config',cfg,'Status',"TEST FIXTURE", ...
    'StartGains',struct('MC_ROLLRATE_D',0.003),'BestGains',struct('MC_ROLLRATE_D',0.0024), ...
    'FigureFiles',strings(0,1),'ReportFiles',strings(0,1));
Session.Results=table([1;2],[10;8],[10;8],'VariableNames',{'Run','Score','BestScore'});
Session.GainHistory=table([0;1;2],repmat("Roll rate",3,1),repmat("MC_ROLLRATE_D",3,1), ...
    [.003;.0036;.0024],[.003;.003;.0024],["INITIAL";"REJECTED";"ACCEPTED"], ...
    [NaN;10;8],[.15;.15;.15], ...
    'VariableNames',{'Run','Stage','Parameter','TrialValue','AcceptedValue','Decision','Score','Amplitude'});
Session.StageResults=table("Roll rate",2,2,8,"TEST FIXTURE", ...
    'VariableNames',{'Stage','Trials','ValidTrials','BestScore','StopReason'});
Session=save_px4_tuning_session(Session);
assert(isfile(fullfile(folder,'session.mat'))&&isfile(fullfile(folder,'results.mat')));
assert(isfile(fullfile(folder,'GainHistory.fig'))&&isempty(dir(fullfile(folder,'*.html'))));
copy=tempname;copyfile(folder,copy);
loaded=load_px4_tuning_session(copy,ShowFigures=false);
assert(isequaln(loaded.Results,Session.Results)&&isequaln(loaded.GainHistory,Session.GainHistory));
assert(loaded.Folder==string(copy),'Copied session folders must load at their new location.');
f=openfig(fullfile(copy,'GainHistory.fig'),'new','invisible');
assert(~isempty(findall(f,'Type','line')));close(f);
loaded.Status="RUNNING";save_px4_tuning_session(loaded);
px4_session_cleanup(char(copy),{'PX4_Nonexistent_Test_Model'});
data=load(fullfile(copy,'session.mat'),'Session');assert(data.Session.Status=="INTERRUPTED");
fprintf('MATLAB_SESSION_TESTS_PASS: selection, 5-valid-trial stagnation, ownership, native figures/tables and relocated reload.\n');
end
