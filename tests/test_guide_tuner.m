function test_guide_tuner()
%TEST_GUIDE_TUNER Offline checks of the multi-axis tuner (guide and RMSE methods). No simulation.
%   test_guide_tuner() checks the multi-axis step schedule, the trajectory, the loop-group
%   configuration, the guide decisions and the RMSE search on synthetic responses, and runs the
%   whole runner (run_px4_guide_tuning) for both methods with synthetic flights.
project=fileparts(fileparts(mfilename('fullpath')));
addpath(project);project_paths(project);
checkSegments();checkTrajectory();checkConfiguration();checkDecisions();checkRmseSearch();
checkRunnerDryRun(project);checkRmseRunnerDryRun(project);
fprintf('test_guide_tuner: all checks passed.\n');
end

%% RMSE search against a synthetic score surface -----------------------------------
function checkRmseSearch()
d=px4_tuning_parameters("Default");[~,G]=px4_guide_configuration("all");
% Roll rate: optimum K = 1.2, D = 0.002, I = 0.25; oscillation above K = 1.5.
A=driveRmse(G(1),1,d,@(g)rmseMet(rollScore(g),rollOsc(g)));
assert(abs(log(A.Gains.MC_ROLLRATE_K/1.2))<log(1.15),'RMSE roll K: %g',A.Gains.MC_ROLLRATE_K);
assert(abs(log(A.Gains.MC_ROLLRATE_D/0.002))<log(1.3),'RMSE roll D: %g',A.Gains.MC_ROLLRATE_D);
assert(A.Gains.MC_ROLLRATE_K<=1.5,'RMSE search accepted an oscillating K.');
% Probe and back-off scale only K for rate loops.
[~,p]=px4_rmse_axis("probe",A,1.3);
assert(abs(p.MC_ROLLRATE_K-px4grid("MC_ROLLRATE_K",1.3*A.Gains.MC_ROLLRATE_K))<1e-12&&p.MC_ROLLRATE_D==A.Gains.MC_ROLLRATE_D);
B=px4_rmse_axis("backoff",A,0.85);assert(abs(B.Gains.MC_ROLLRATE_K-px4grid("MC_ROLLRATE_K",0.85*A.Gains.MC_ROLLRATE_K))<1e-12);
% Velocity: P, I and D are all searched; P optimum 4.5 (like the earlier RMSE campaign).
A=driveRmse(G(3),1,d,@(g)rmseMet((log(g.MPC_XY_VEL_P_ACC/4.5))^2+(log(g.MPC_XY_VEL_D_ACC/0.2))^2+0.1,0.05));
assert(abs(log(A.Gains.MPC_XY_VEL_P_ACC/4.5))<log(1.15),'RMSE XY vel P: %g',A.Gains.MPC_XY_VEL_P_ACC);
% A score inside the repeat noise is not accepted.
A=px4_rmse_axis("init",[],G(4),1,d);
A=px4_rmse_axis("baseline",A,[rmseMet(1,0.05) rmseMet(1.1,0.05) rmseMet(0.9,0.05)]);
[A,~]=px4_rmse_axis("propose",A);[A,~]=px4_rmse_axis("observe",A,rmseMet(0.95,0.05));
assert(A.Gains.MPC_XY_P==d.MPC_XY_P,'A change inside the noise margin was accepted.');
end

function s=rollScore(g)
s=(log(g.MC_ROLLRATE_K/1.2))^2+(log(g.MC_ROLLRATE_D/0.002))^2+0.3*(log(g.MC_ROLLRATE_I/0.25))^2+0.05;
end
function o=rollOsc(g),o=0.05+0.4*(g.MC_ROLLRATE_K>1.5);end

function A=driveRmse(G,j,tune,model)
A=px4_rmse_axis("init",[],G,j,tune);
b=model(A.Gains);A=px4_rmse_axis("baseline",A,[b b b]);
for k=1:60
    [A,g]=px4_rmse_axis("propose",A);
    if A.Pending.Kind=="hold",return;end
    [A,~]=px4_rmse_axis("observe",A,model(g));
end
error('Synthetic RMSE search for %s did not finish.',A.Key);
end

function m=rmseMet(score,oscillation)
m=struct('Valid',true,'Unstable',false,'Overshoot',0,'Tracking',score,'LateError',0, ...
    'OvershootRMS',0,'Oscillation',oscillation,'MotorVariation',0,'Score',score,'RMSEScore',score,'Usable',true);
end

function checkRmseRunnerDryRun(project)
here=pwd;back=onCleanup(@()cd(here));cd(project);
out=fullfile(tempdir,"rmse_dryrun_"+string(datetime('now','Format','yyyyMMdd_HHmmss_SSS')));mkdir(out);
initialize_px4_tuning_session(project);
S=run_px4_guide_tuning("all",Method="rmse",FlightFunction=@fakeRmseFlight,ShowFigures=false,OutputRoot=out);
assert(S.Status=="COMPLETE",'RMSE dry run status %s: %s',S.Status,S.FatalError);
assert(S.Method=="rmse"&&contains(S.Folder,"_rmse_tune"));
assert(height(S.Final)==10&&all(S.Final.Valid),'RMSE final check incomplete.');
probes=S.Flights(startsWith(S.Flights.Purpose,"robustness probe"),:);
assert(~isempty(probes),'No robustness probe was flown.');
K=S.BestGains.MC_ROLLRATE_K;assert(1.3*K<=1.5+1e-9,'Final roll K %g does not pass a x1.3 probe.',K);
assert(any(contains(S.Decisions,"robustness probe x1.3 passed")));
fprintf('RMSE dry run: %d flights (%d robustness probes), roll K %.3g.\n',max(S.Flights.Flight),numel(unique(probes.Flight)),K);
end

function [M,status,wall,diagnosis]=fakeRmseFlight(~,G,tune,~)
n=numel(G.Members);M=cell(1,n);diagnosis=cell(1,n);status=repmat("VALID",1,n);wall=1;
for j=1:n
    key=G.Members(j).Key;
    switch key
        case {"rollrate","pitchrate"},a=upper(extractBefore(key,"rate"));
            g=struct('MC_ROLLRATE_K',tune.("MC_"+a+"RATE_K"),'MC_ROLLRATE_D',tune.("MC_"+a+"RATE_D"),'MC_ROLLRATE_I',tune.("MC_"+a+"RATE_I"));
            s=rollScore(g);o=rollOsc(g);
        case "horizontalvelocity",s=(log(tune.MPC_XY_VEL_P_ACC/4.5))^2+0.1;o=0.05;
        otherwise,s=0.2;o=0.05;
    end
    M{j}=struct('Score',s,'LegacyTrackingScore',s,'JointFeatures',struct('Tracking',s,'LateError',0, ...
        'OvershootRMS',0,'Oscillation',o,'MotorVariation',0,'Score',s,'Usable',true));
    diagnosis{j}=struct('Code',"VALID",'Response',struct('OvershootPercent',0));
end
end

%% Whole runner, offline: synthetic flights replace the simulator ------------------
function checkRunnerDryRun(project)
here=pwd;back=onCleanup(@()cd(here));cd(project);
out=fullfile(tempdir,"guide_dryrun_"+string(datetime('now','Format','yyyyMMdd_HHmmss_SSS')));mkdir(out);
initialize_px4_tuning_session(project);
S=run_px4_guide_tuning("all",FlightFunction=@fakeFlight,ShowFigures=false,OutputRoot=out);
assert(S.Status=="COMPLETE",'Dry run status %s: %s',S.Status,S.FatalError);
assert(height(S.Final)==10&&all(S.Final.Valid),'Final check incomplete.');
assert(abs(S.BestGains.MC_ROLLRATE_K-1.2^3*0.75)<1e-9&&abs(S.BestGains.MC_YAWRATE_I-0.1*1.5^2)<1e-9);
assert(isfile(fullfile(S.Folder,'guide_flights.csv'))&&isfile(fullfile(S.Folder,'guide_decisions.txt')) ...
    &&isfile(fullfile(S.Folder,'session.mat'))&&isfile(fullfile(S.Folder,'GuideGainHistory.png')));
fprintf('Dry run: %d flights, %d decisions logged.\n',max(S.Flights.Flight),numel(S.Decisions));
end

function [M,status,wall,diagnosis]=fakeFlight(~,G,tune,~)
n=numel(G.Members);M=cell(1,n);diagnosis=cell(1,n);status=repmat("VALID",1,n);wall=1;
for j=1:n
    x=synthetic(G.Members(j).Key,tune);
    M{j}=struct('Score',x.Score,'JointFeatures',struct('Tracking',x.Tracking,'LateError',x.LateError, ...
        'OvershootRMS',0,'Oscillation',x.Oscillation,'MotorVariation',0,'Score',x.Score,'Usable',true));
    diagnosis{j}=struct('Code',"VALID",'Response',struct('OvershootPercent',x.Overshoot));
end
end

function m=synthetic(key,g)
switch key
    case {"rollrate","pitchrate"}
        a=upper(extractBefore(key,"rate"));D=g.("MC_"+a+"RATE_D");K=g.("MC_"+a+"RATE_K");
        m=met(3.5+(0.0024-D)/0.0024*25,0.2/K,0.05+0.25*(D>=0.0029)+0.35*(K>1.6),0.02);
    case "yawrate"
        m=met(2+10*(g.MC_YAWRATE_I-0.1),0.2/g.MC_YAWRATE_K,0.05+0.35*(g.MC_YAWRATE_K>1.5),0.02/g.MC_YAWRATE_I);
    case {"rollattitude","pitchattitude"}
        P=g.("MC_"+upper(extractBefore(key,"attitude"))+"_P");m=met(max(0,(P-10)*3),1/P,0.05,0.01);
    case "yawattitude"
        m=met(max(0,(g.MC_YAW_P-4)*5),1/g.MC_YAW_P,0.05,0.01);
    case "horizontalvelocity"
        m=met(max(0,(g.MPC_XY_VEL_P_ACC-3)*5)+20*(g.MPC_XY_VEL_I_ACC>0.45),1/g.MPC_XY_VEL_P_ACC,0.05,0.01, ...
            1/g.MPC_XY_VEL_P_ACC+2*g.MPC_XY_VEL_D_ACC);
    case "verticalvelocity"
        m=met(max(0,(g.MPC_Z_VEL_P_ACC-6)*3),1/g.MPC_Z_VEL_P_ACC,0.05,0.01);
    case "horizontalposition"
        m=met(0,0.1,0.02,0.2/g.MPC_XY_P);
    case "verticalposition"
        m=met(0,0.1,0.02,0.2/g.MPC_Z_P);
end
end

%% Multi-axis schedule -----------------------------------------------------
function checkSegments()
% Legacy single-axis codes are unchanged.
[a,ts,amp]=px4_tuning_segment(12.3,uint8(1),uint8(2),single(0.15));
assert(a==2&&ts==12.3&&amp==single(0.15));
% Rates: roll, pitch, yaw in sequence, ten signed pulses each.
[~,~,d]=px4_tuning_step_windows(1);code=uint8(8+7);t=(0:0.005:3*d+5)';
axisNow=zeros(size(t));cmd=zeros(size(t));
for k=1:numel(t)
    [ax,st,A]=px4_tuning_segment(t(k),uint8(1),code,single(0.15));
    axisNow(k)=ax;cmd(k)=px4_tuning_step_command(st,uint8(1),A);
end
for k=1:3
    m=axisNow==k;assert(all(t(m)>=(k-1)*d-1e-9&t(m)<k*d+1e-9),'Axis %d outside its segment.',k);
    c=cmd(m);on=find(diff([0;abs(c)>0])==1);assert(numel(on)==10,'Axis %d has %d pulses.',k,numel(on));
    assert(all(abs(abs(c(abs(c)>0))-0.15)<1e-6));
    s=sign(c(on));assert(isequal(s',repmat([1 -1],1,5)),'Pulse signs wrong for axis %d.',k);
end
assert(all(axisNow(t>=3*d)==0)&&all(cmd(t>=3*d)==0),'Excitation after the last segment.');
[a,~]=px4_tuning_segment(-0.5,uint8(1),code,single(0.15));assert(a==0);
% Velocity: horizontal x then vertical z; vertical amplitude 0.6 (= 0.75 x 0.8).
[~,~,d]=px4_tuning_step_windows(3);
[a1,~,A1]=px4_tuning_segment(5,uint8(3),uint8(8+1+4),single(0.8));
[a2,s2,A2]=px4_tuning_segment(d+5,uint8(3),uint8(8+1+4),single(0.8));
assert(a1==1&&abs(A1-0.8)<1e-6&&a2==3&&abs(A2-0.6)<1e-6&&abs(s2-5)<1e-9);
end

function checkTrajectory()
plan=flight_plan("square");plan.Size=0;plan.Altitude=10;home=single([0 0 -10]);
[~,~,d]=px4_tuning_step_windows(3);[w]=px4_tuning_step_windows(3);
% Inside first pulse of the vertical segment: only velocity z is commanded (NED, sign from schedule).
[~,v]=px4_tuning_trajectory(d+w(1,1)+0.5,3,uint8(13),single(0.8),home,plan,40);
assert(v(1)==0&&v(2)==0&&abs(abs(v(3))-0.6)<1e-6);
[p]=px4_tuning_trajectory(w(1,1)+0.5,4,uint8(13),single(0.8),home,plan,40);
assert(abs(p(1)-0.8)<1e-6&&p(3)==single(-10));
end

function checkConfiguration()
[cfg,G]=px4_guide_configuration("all");
assert(isequal([G.Key],["rates","attitude","velocity","position"]));
assert(isequal(double([G.Axis]),[15 15 13 13]));
for g=G
    [~,~,d]=px4_tuning_step_windows(g.Mode);
    assert(isequal([g.Members.SegmentOffset],(0:numel(g.Axes)-1)*d));
    assert(g.SimTime>=cfg.TestStart+numel(g.Axes)*d,'SimTime too short for %s.',g.Key);
end
assert(abs(G(3).Members(2).Amplitude-0.6)<1e-6&&abs(G(4).Members(1).Amplitude-0.8)<1e-6);
end

%% Guide decisions against simple synthetic responses ---------------------------
function checkDecisions()
d=px4_tuning_parameters("Default");[~,G]=px4_guide_configuration("all");
% Rates: roll oscillates at Default D (osc_fix), is then overdamped (pd_balance lowers D),
% then oscillates above K=1.6 (K is backed off to 0.75 x onset).
roll=@(g)met(3.5+(0.0024-g.MC_ROLLRATE_D)/0.0024*25,0.2/g.MC_ROLLRATE_K, ...
    0.05+0.25*(g.MC_ROLLRATE_D>=0.0029)+0.35*(g.MC_ROLLRATE_K>1.6),0.02);
A=drive(G(1),1,d,roll);
% Expected values are rounded to the PX4 resolution of each gain (px4grid), as the tuner flies them.
assert(abs(A.Gains.MC_ROLLRATE_D-px4grid("MC_ROLLRATE_D",0.0027*0.8))<1e-9,'Roll D: %g',A.Gains.MC_ROLLRATE_D);
assert(abs(A.Gains.MC_ROLLRATE_K-px4grid("MC_ROLLRATE_K",1.2^3*0.75))<1e-9,'Roll K: %g',A.Gains.MC_ROLLRATE_K);
% Yaw: K ramp, then I raised until it tracks the setpoint (late error <= 0.1).
yaw=@(g)met(2+10*(g.MC_YAWRATE_I-0.1),0.2/g.MC_YAWRATE_K,0.05+0.35*(g.MC_YAWRATE_K>1.5),0.02/g.MC_YAWRATE_I);
A=drive(G(1),3,d,yaw);
assert(abs(A.Gains.MC_YAWRATE_K-px4grid("MC_YAWRATE_K",1.2^3*0.75))<1e-9,'Yaw K: %g',A.Gains.MC_YAWRATE_K);
assert(abs(A.Gains.MC_YAWRATE_I-px4grid("MC_YAWRATE_I",0.1*1.5^2))<1e-9,'Yaw I: %g',A.Gains.MC_YAWRATE_I);
% Attitude: P raised until overshoot; bound 12 overshoots, previous value kept.
att=@(g)met(max(0,(g.MC_ROLL_P-10)*3),1/g.MC_ROLL_P,0.05,0.01);
A=drive(G(2),1,d,att);assert(abs(A.Gains.MC_ROLL_P-px4grid("MC_ROLL_P",6.5*1.2^3))<1e-9,'Roll P: %g',A.Gains.MC_ROLL_P);
% Velocity: overdamped default -> lower D kept; P raised until overshoot; PID scale overshoots at once.
vel=@(g)met(max(0,(g.MPC_XY_VEL_P_ACC-3)*5)+20*(g.MPC_XY_VEL_I_ACC>0.45)+ (g.MPC_XY_VEL_D_ACC>0.15)*0, ...
    1/g.MPC_XY_VEL_P_ACC,0.05,0.01,1/g.MPC_XY_VEL_P_ACC+2*g.MPC_XY_VEL_D_ACC);
A=drive(G(3),1,d,vel);
assert(abs(A.Gains.MPC_XY_VEL_D_ACC-0.1)<1e-9,'XY vel D: %g',A.Gains.MPC_XY_VEL_D_ACC);
assert(abs(A.Gains.MPC_XY_VEL_P_ACC-px4grid("MPC_XY_VEL_P_ACC",1.8*1.2^4))<1e-9,'XY vel P: %g',A.Gains.MPC_XY_VEL_P_ACC);
% Position: P raised while the hold improves; stops at the published bound 2.0.
pos=@(g)met(0,0.1,0.02,0.2/g.MPC_XY_P);
A=drive(G(4),1,d,pos);assert(abs(A.Gains.MPC_XY_P-2)<1e-9,'XY P: %g',A.Gains.MPC_XY_P);
end

function v=px4grid(name,x)
% x rounded to the PX4 resolution of gain NAME (px4_controller_gain_limits).
[~,~,res]=px4_controller_gain_limits(name);v=round(x,round(-log10(res)));
end

function A=drive(G,j,tune,model)
A=px4_guide_axis("init",[],G,j,tune);
b=model(A.Gains);A=px4_guide_axis("baseline",A,[b b b]);
for k=1:60
    [A,g]=px4_guide_axis("propose",A);
    if A.Pending.Kind=="hold",return;end
    [A,~]=px4_guide_axis("observe",A,model(g));
end
error('Synthetic procedure for %s did not finish.',A.Key);
end

function m=met(overshoot,tracking,oscillation,late,score)
if nargin<5,score=tracking+oscillation+late;end
m=struct('Valid',true,'Unstable',false,'Overshoot',overshoot,'Tracking',tracking,'LateError',late, ...
    'OvershootRMS',0,'Oscillation',oscillation,'MotorVariation',0,'Score',score,'Usable',true);
end
