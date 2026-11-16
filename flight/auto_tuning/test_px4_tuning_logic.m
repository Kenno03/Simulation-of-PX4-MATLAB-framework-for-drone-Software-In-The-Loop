function test_px4_tuning_logic()
%TEST_PX4_TUNING_LOGIC Synthetic scoring tests, not aircraft simulation.
cfg=struct('HomeNED',single([0 0 -10]),'TakeoffTolerance',1, ...
    'MinAltitude',2,'MinTestAltitude',2,'MaxBodyRate',5,'MaxRecoveryRate',5,'ObjectiveVersion',"joint_response_v1");
plan=flight_plan("square");plan.Size=0;plan.Altitude=10;
for mode=1:4
    clear px4_tuning_schedule;
    [m,relative]=px4_tuning_schedule(0,mode,40,0,0,plan,2,1);
    assert(m==4 && relative<0);
    [m,relative]=px4_tuning_schedule(40,mode,40,0,0,plan,2,1);
    assert(m==4 && relative<0,'Grounded vehicle must not receive excitation.');
    px4_tuning_schedule(41,mode,40,-3,0.5,plan,2,1);
    [m,relative]=px4_tuning_schedule(44.99,mode,40,-3,0.5,plan,2,1);
    assert(m==4 && relative<0,'Settling duration must be complete.');
    [m,relative]=px4_tuning_schedule(45,mode,40,-3,0.5,plan,2,1);
    assert(m==mode && relative==0);
end
fprintf('TAKEOFF_SETTLING_GATE_PASS\n');
for mode=1:4
    for axis=1:3
        S=struct('Mode',uint8(mode),'Axis',uint8(axis),'Amplitude',single(0.15));
        [plant,controller]=syntheticData(S);
        M=score_px4_tuning_candidate(plant,controller,S,cfg);
        assert(M.RMSE<1e-5&&M.Score<1e-5&&M.JointFeatures.Usable,'Perfect synthetic tracking must pass the new objective.');
    end
end
S=struct('Mode',uint8(1),'Axis',uint8(2),'Amplitude',single(0.15));
[plant,controller]=syntheticData(S);
e=controller.getElement('TUNING_CMD_RATE_PITCH');
e.Values.Data=-e.Values.Data;
controller=controller.setElement(find(strcmp(controller.getElementNames,'TUNING_CMD_RATE_PITCH')),e);
rejected=false;
try,score_px4_tuning_candidate(plant,controller,S,cfg);catch ME
    rejected=contains(ME.message,'amplitude/sign');
end
assert(rejected,'An inverted/wrong reference must be rejected.');
[plant,controller]=syntheticData(S);
e=controller.getElement('TUNING_CMD_RATE_PITCH');
e.Values=e.Values.resample((0:0.02:48)');
controller=controller.setElement(find(strcmp(controller.getElementNames,'TUNING_CMD_RATE_PITCH')),e);
rejected=false;
try,score_px4_tuning_candidate(plant,controller,S,cfg);catch ME
    rejected=contains(ME.message,'Fewer than 5');
end
assert(rejected,'A truncated test/recovery window must be rejected.');
% Incomplete schedules are usable with 5+ complete response/recovery windows.
for mode=1:4
    partial=struct('Mode',uint8(mode),'Axis',uint8(1),'Amplitude',single(.15));
    for wanted=[4 5 9]
        [plant,controller]=syntheticData(partial);
        [windows,~,duration]=px4_tuning_step_windows(mode);
        recovery=duration-windows(end,2);
        stopAt=40+windows(wanted,2)+recovery+.05;
        commandNames=["TUNING_CMD_RATE_ROLL","TUNING_CMD_ATTITUDE","TUNING_CMD_VELOCITY","TUNING_CMD_POSITION"];
        name=commandNames(mode);element=controller.getElement(name);
        element.Values=element.Values.resample((0:.02:stopAt)');
        controller=controller.setElement(find(strcmp(controller.getElementNames,name)),element);
        if wanted>=5
            metrics=score_px4_tuning_candidate(plant,controller,partial,cfg);
            assert(metrics.StepCount==wanted&&metrics.RMSE<1e-5);
        else
            rejected=false;
            try,score_px4_tuning_candidate(plant,controller,partial,cfg);catch ME
                rejected=contains(ME.message,'Fewer than 5');
            end
            assert(rejected,'Four usable steps must be rejected.');
        end
    end
end
fprintf('MINIMUM_STEP_POLICY_PASS: 5 and 9 accepted; 4 rejected in all four modes.\n');
% A fixed plant/EKF heading alignment must not be mistaken for yaw-P tracking.
yawStage=struct('Mode',uint8(2),'Axis',uint8(3),'Amplitude',single(.15));
[plant,controller]=syntheticData(yawStage);
element=plant.getElement('Drone State');element.Values.Eta_NED.Data(:,1)=element.Values.Eta_NED.Data(:,1)+.173;
plant=plant.setElement(find(strcmp(plant.getElementNames,'Drone State')),element);
metrics=score_px4_tuning_candidate(plant,controller,yawStage,cfg);assert(metrics.RMSE<1e-5);
aligned=px4_tuning_trial_data(plant,controller,yawStage,cfg);
assert(abs(median(aligned.TruthMeasured-aligned.Measured)-.173)<1e-6);
fprintf('YAW_FEEDBACK_ALIGNMENT_TEST_PASS: controller feedback scored; absolute truth preserved.\n');
fprintf('SCORING_SYNTHETIC_PASS: all 4 modes x 3 axes; wrong sign and truncated logs rejected.\n');
cfg.TestStart=40;
cfg.SimTime=140;cfg.MaxReadyClimbRate=1;cfg.FlightPlan=plan;
S.Name="Synthetic pitch rate fixture";
[plant,controller]=syntheticData(S);
folder=tempname;mkdir(folder);
d=write_px4_trial_report(plant,controller,S,cfg,"VALID",folder,struct('ID',"Default"));
assert(d.Code=="VALID" && isfile(fullfile(folder,'NormalizedStepResponse.png')));
item=load(fullfile(folder,'TrialReport.mat'),'Report');content=item.Report;
assert(content.StepCount==10 && content.ResponseMetricsUsable);
assert(~isfield(content,'ReportDataIssue') && isfile(fullfile(folder,'RawResponse.png')));
e=plant.getElement('Drone State');
e.Values.Omega_nav_ned.Data=e.Values.Omega_nav_ned.Data*40;
plant=plant.setElement(find(strcmp(plant.getElementNames,'Drone State')),e);
d=write_px4_trial_report(plant,controller,S,cfg,"INVALID: Body-rate safety limit crossed.",folder,struct('ID',"Default"));
assert(d.AllowAmplitudeRetry && d.Code=="EXCITATION_OR_RECOVERY_UNSTABLE");
d=write_px4_trial_report(plant,controller,S,cfg,"INVALID: Reference does not match stage amplitude/sign.",folder,struct('ID',"Default"));
assert(~d.AllowAmplitudeRetry && d.Code=="REFERENCE_OR_LOGGING_FAILURE");
d=write_px4_trial_report([],[],S,cfg,"ERROR: connection lost",folder,struct('ID',"Default"));
assert(~d.AllowAmplitudeRetry && d.Code=="EXECUTION_FAILURE");
fprintf('TRIAL_REPORT_AND_RETRY_DIAGNOSIS_PASS\n');
end

function [plant,controller]=syntheticData(S)
t=(0:0.02:140)'; measuredTime=t+2; relative=t-40;
A=double(S.Amplitude); cmd=zeros(size(t));
for k=1:numel(t),cmd(k)=double(px4_tuning_step_command(relative(k),S.Mode,S.Amplitude));end
n=numel(t);axis=double(S.Axis);
pos=repmat([0 0 -10],n,1);vel=zeros(n,3);eta=zeros(n,3);omega=zeros(n,3);
controller=Simulink.SimulationData.Dataset;
mode=repmat(double(S.Mode),n,1);mode(t<40)=4;
controller=add(controller,'TUNING_MODE',timeseries(mode,t));
controller=add(controller,'TUNING_TEST_TIME',timeseries(relative,t));
controller=add(controller,'<timestamp>',timeseries(uint64(measuredTime*1e6),t));
switch double(S.Mode)
    case 1
        names=["ROLL","PITCH","YAW"];
        controller=add(controller,'TUNING_CMD_RATE_'+names(axis),timeseries(cmd,t));
        omega(:,axis)=cmd;
    case 2
        eul=zeros(n,3);eul(:,axis)=cmd;
        q=zeros(n,4);q(:,1)=cos(cmd/2);q(:,axis+1)=sin(cmd/2);
        controller=add(controller,'TUNING_CMD_ATTITUDE',timeseries(q,t));
        controller=add(controller,'<heading>',timeseries(eul(:,3),t));
        eta=eul(:,[3 2 1]); % saved plant bus uses yaw,pitch,roll
    case 3
        vel(:,axis)=cmd;
        controller=add(controller,'TUNING_CMD_VELOCITY',timeseries(vel,t));
    case 4
        pos(:,axis)=pos(:,axis)+cmd;
        controller=add(controller,'TUNING_CMD_POSITION',timeseries(pos,t));
end
R=[cos(pi/3) sin(pi/3) 0;-sin(pi/3) cos(pi/3) 0;0 0 1];
nav=omega*R;
state=struct('S_NED',timeseries(pos,measuredTime), ...
    'Sdot_NED',timeseries(vel,measuredTime), ...
    'Eta_NED',timeseries(eta,measuredTime), ...
    'Omega_nav_ned',timeseries(nav,measuredTime), ...
    'R_ned_to_frd',timeseries(repmat(R,1,1,n),measuredTime));
plant=Simulink.SimulationData.Dataset;
plant=add(plant,'Drone State',state);
for k=1:4,plant=add(plant,"Motor "+k,timeseries(0.3*ones(n,1),measuredTime));end
end

function dataset=add(dataset,name,value)
signal=Simulink.SimulationData.Signal;
signal.Name=char(name);signal.Values=value;
dataset=dataset.addElement(signal);
end
