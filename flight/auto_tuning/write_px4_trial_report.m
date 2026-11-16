function diagnosis=write_px4_trial_report(plant,controller,S,cfg,status,folder,tune)
%WRITE_PX4_TRIAL_REPORT Save MATLAB figures, data and diagnostics in the session folder.
% Direct normalization of commanded steps; not PX4 Flight Review identification.
if ~isfolder(folder),mkdir(folder);end
prefix='';if isfield(cfg,'TrialPrefix'),prefix=char(cfg.TrialPrefix);end
diagnosis=struct('Code',"EXECUTION_FAILURE",'Summary',string(status), ...
    'NextAction',"Inspect startup/connection diagnostics; do not change gains.", ...
    'AllowAmplitudeRetry',false,'ReportFolder',string(folder));
report=struct('Stage',string(S.Name),'Mode',double(S.Mode),'Axis',double(S.Axis), ...
    'Amplitude',double(S.Amplitude),'Status',string(status), ...
    'Method',"Measured step-aligned responses, baseline-subtracted and divided by the signed command step. Spread is pulse min/max, not a confidence interval.", ...
    'GainValues',tune,'ExpectedStepCount',10,'MinimumStepCount',5,'StepCount',0,'ResponseMetricsUsable',false);
try
    assert(~isempty(plant) && ~isempty(controller),'No complete fresh plant/controller data.');
    D=px4_tuning_trial_data(plant,controller,S,cfg);
    [clockTime,relative]=matrix(controller.getElement('TUNING_TEST_TIME').Values);
    clockTime=clockTime+D.ClockOffset;
    released=isfinite(relative)&relative>=0;
    available=string(controller.getElementNames);
    if any(available=="PX4_PARAMETER_WRITE_TRIGGER")
        writeTrigger=controller.getElement('PX4_PARAMETER_WRITE_TRIGGER').Values;
        report.ParameterWriteTriggerSeen=any(writeTrigger.Data~=0,'all');
    end
    if any(released),testStart=median(clockTime(released)-relative(released));
    else,testStart=cfg.TestStart+D.ClockOffset;end
    % Multi-axis flights: this report covers one axis segment.
    if isfield(S,'SegmentOffset'),testStart=testStart+double(S.SegmentOffset);end
    report.TestStartPlantSeconds=testStart;
    report.MeasurementSource=D.MeasurementSource;
    if double(S.Mode)==2&&double(S.Axis)==3
        truth=interp1(D.TruthMeasuredTime,D.TruthMeasured,D.MeasuredTime,'linear',NaN);
        before=D.MeasuredTime>=testStart-2&D.MeasuredTime<testStart;
        difference=atan2(sin(truth-D.Measured),cos(truth-D.Measured));
        report.PreTestYawTruthMinusFeedback=median(difference(before),'omitnan');
    end
    report.TargetAltitude=-double(cfg.HomeNED(3));
    report.MinimumTestAltitude=cfg.MinTestAltitude;
    report.AltitudeAtTestStart=interp1(D.PlantTime,D.Altitude,testStart,'linear',NaN);
    on=D.RateTime>=testStart;
    report.MaxBodyRate=max(abs(D.BodyRates(on,:)),[],'all');
    angles=D.EulerTime>=testStart;
    report.MaxTilt=max(abs(D.Euler(angles,1:2)),[],'all');
    before=D.RateTime>=testStart-2 & D.RateTime<testStart;
    report.PreTestMaxBodyRate=max(abs(D.BodyRates(before,:)),[],'all');
    before=D.EulerTime>=testStart-2 & D.EulerTime<testStart;
    report.PreTestMaxTilt=max(abs(D.Euler(before,1:2)),[],'all');
    active=D.PlantTime>=testStart;
    report.MinAltitude=min(D.Altitude(active));
    report.MaxMotor=0;
    for k=1:4
        [tm,u]=matrix(plant.getElement("Motor "+k).Values);
        report.MaxMotor=max(report.MaxMotor,max(u(tm>=testStart,:),[],'all'));
    end
    estimatedTime=[];estimatedHeight=[];verticalTime=[];verticalSpeed=[];
    try
        names=string(controller.getElementNames);
        assert(any(names=="<z>")&&any(names=="<vz>"),'Estimator signals not logged.');
        [estimatedTime,z]=matrix(controller.getElement('<z>').Values);
        estimatedHeight=-z;
        [verticalTime,vz]=matrix(controller.getElement('<vz>').Values);
        verticalSpeed=-vz;
        vertical=interp1(verticalTime,verticalSpeed,estimatedTime,'previous',NaN);
        after=estimatedTime>=cfg.TestStart;
        report.MaximumEstimatedAltitudeAfterEarliestStart=max(estimatedHeight(after));
        eligible=after & estimatedHeight>cfg.MinTestAltitude & ...
            abs(vertical)<=cfg.MaxReadyClimbRate;
        edges=diff([false;eligible;false]);starts=find(edges==1);ends=find(edges==-1)-1;
        durations=estimatedTime(ends)-estimatedTime(starts);
        report.LongestEstimatedReadinessSeconds=max([0;durations]);
        report.ReadinessNeedsSeconds=cfg.FlightPlan.SettleTime;
        report.MaximumReadyClimbRate=cfg.MaxReadyClimbRate;
    catch
        % Reports retain the raw truth even if estimator fields were not logged.
    end
    diagnosis.Code="TEST_FAILED";
    diagnosis.NextAction="Inspect command/response traces and timing; keep gains fixed until a valid baseline exists.";
    if startsWith(status,"ERROR:")
        diagnosis.Code="EXECUTION_FAILURE";diagnosis.Summary=status;
        diagnosis.NextAction="Inspect execution/connection/scheduler diagnostics; hold gains fixed.";
    elseif isfield(report,'ParameterWriteTriggerSeen')&&~report.ParameterWriteTriggerSeen
        diagnosis.Code="PARAMETER_WRITE_NOT_TRIGGERED";
        diagnosis.Summary="The parameter-write trigger did not execute. Gain/geometry writes cannot be assumed to have occurred.";
        diagnosis.NextAction="Repair the parameter-write clock/trigger before changing gains or excitation.";
    elseif ~any(released)
        diagnosis.Code="TAKEOFF_NOT_READY";
        diagnosis.Summary="The flight did not satisfy the altitude/climb readiness gate; excitation was withheld.";
        if isfield(report,'LongestEstimatedReadinessSeconds')
            diagnosis.Summary=diagnosis.Summary+sprintf( ...
                ' Maximum estimated altitude after %.1f s: %.2f m; longest eligible dwell: %.2f s (needs %.2f s).', ...
                cfg.TestStart, ...
                report.MaximumEstimatedAltitudeAfterEarliestStart, ...
                report.LongestEstimatedReadinessSeconds,report.ReadinessNeedsSeconds);
        end
        diagnosis.NextAction="Review takeoff/arming and the altitude trace; do not adjust rate gains for a test that never started.";
    elseif status=="VALID"
        diagnosis.Code="VALID";diagnosis.Summary="Trial passed scoring and safety checks.";
        diagnosis.NextAction="Compare normalized response and score with trials using the same excitation amplitude.";
    elseif contains(status,'Fewer than 5')
        diagnosis.Code="INSUFFICIENT_STEPS";
        diagnosis.Summary="Fewer than five complete usable steps and recovery windows were recorded.";
        diagnosis.NextAction="Inspect readiness and available test time before changing gains.";
    elseif report.AltitudeAtTestStart<=cfg.MinTestAltitude
        diagnosis.Code="TAKEOFF_NOT_READY";
        diagnosis.Summary="The recorded test began below the minimum test altitude. The readiness gate must suppress such a test.";
        diagnosis.NextAction="Review takeoff/arming and altitude before excitation; keep rate gains fixed.";
    elseif any(contains(status,["amplitude/sign","Wrong controller mode","scheduled pulse","ends before","Missing","not found"]))
        diagnosis.Code="REFERENCE_OR_LOGGING_FAILURE";
        diagnosis.NextAction="Repair reference/timing/logging before changing amplitude or gains.";
    elseif report.PreTestMaxBodyRate>1 || report.PreTestMaxTilt>0.6
        diagnosis.Code="BASELINE_UNSTABLE";
        diagnosis.NextAction="Instability predates excitation. Review takeoff/hover; reducing the tuning step cannot establish the cause.";
    elseif report.MaxBodyRate>cfg.MaxBodyRate || report.MaxTilt>1 || ...
            report.MaxMotor>=0.99 || report.MinAltitude<cfg.MinAltitude
        diagnosis.Code="EXCITATION_OR_RECOVERY_UNSTABLE";
        diagnosis.Summary="Instability/saturation appeared during the test or recovery. This does not prove excessive excitation caused it.";
        diagnosis.AllowAmplitudeRetry=true;
        diagnosis.NextAction="Retry a baseline with half the excitation amplitude and identical gains, within the original trial limit.";
    end
    [usable,stepWindows]=px4_tuning_completed_steps(D,S,testStart);
    report.CompletedStepCount=nnz(usable);
    r=D.Reference-D.Nominal;
    [time,ix]=unique(D.Time);r=r(ix);
    y=interp1(D.MeasuredTime,D.Measured-D.Nominal,time,'linear',NaN);
    changes=[false;abs(diff(r))>0.25*double(S.Amplitude)];
    old=[r(1);r(1:end-1)];
    starts=find(changes & time>=testStart & abs(r)>0.4*double(S.Amplitude) & ...
        abs(old)<0.2*double(S.Amplitude));
    % Show the full commanded hold. A fixed short crop hides slow settling
    % and later overshoot, especially in velocity and position loops.
    horizon=min(stepWindows(:,2)-stepWindows(:,1))-.05;
    dt=max(0.002,median(diff(time)));gridTime=(0:dt:horizon)';
    traces=[];commands=[];
    for k=1:numel(starts)
        at=starts(k);onset=time(at);delta=r(at)-old(at);
        if ~any(usable & abs(stepWindows(:,1)-(onset-testStart))<.05),continue;end
        if onset+horizon>time(end),continue;end
        baseline=time>=onset-0.15 & time<onset-0.02 & isfinite(y);
        if nnz(baseline)<3,continue;end
        base=median(y(baseline));
        segment=interp1(time,y,onset+gridTime,'linear',NaN);
        command=interp1(time,r,onset+gridTime,'previous',NaN);
        if any(~isfinite(segment)) || any(abs(command-r(at))>0.1*abs(delta)),continue;end
        traces(:,end+1)=(segment-base)/delta; %#ok<AGROW>
        commands(:,end+1)=(command-old(at))/delta; %#ok<AGROW>
    end
    report.StepCount=size(traces,2);
    if status=="VALID"
        diagnosis.Summary=sprintf('Trial passed scoring and safety checks using %d of 10 target steps (minimum 5).',report.CompletedStepCount);
    end
    report.RiseTime10to90=NaN;report.SettlingTime5Percent=NaN;
    report.OvershootPercent=NaN;report.EndWindowGain=NaN;
    blue=[0.12 0.62 0.85];
    f=figure('Visible','off','Color','w','Position',[100 100 1100 660]);
    guard=onCleanup(@()close(f)); %#ok<NASGU>
    ax=axes(f,'Position',[0.09 0.23 0.86 0.64]);hold(ax,'on');
    if ~isempty(traces)
        average=mean(traces,2);lo=min(traces,[],2);hi=max(traces,[],2);
        fill(ax,[gridTime;flipud(gridTime)],[lo;flipud(hi)],blue, ...
            'FaceAlpha',0.16,'EdgeColor','none','DisplayName','Pulse range');
        plot(ax,gridTime,traces,'Color',[0.65 0.83 0.92],'HandleVisibility','off');
        plot(ax,gridTime,mean(commands,2),'Color',[0.18 0.22 0.27], ...
            'LineWidth',1.3,'DisplayName','Normalized command');
        plot(ax,gridTime,average,'Color',blue,'LineWidth',2.5,'DisplayName','Mean measured response');
        yline(ax,1,'-','Color',[0.25 0.28 0.34],'HandleVisibility','off');
        a=find(average>=0.1,1);b=find(average>=0.9,1);
        if ~isempty(a)&&~isempty(b)&&b>=a&&average(1)<0.1,report.RiseTime10to90=gridTime(b)-gridTime(a);end
        outside=find(abs(average-1)>0.05,1,'last');
        if isempty(outside),report.SettlingTime5Percent=0;
        elseif outside<numel(gridTime),report.SettlingTime5Percent=gridTime(outside+1);end
        report.OvershootPercent=max(0,max(average)-1)*100;
        report.EndWindowGain=mean(average(max(1,round(0.8*numel(average))):end));
        report.ObservedPeakNormalized=max(average);
        report.ResponseMetricsUsable=status=="VALID" && report.StepCount>=report.MinimumStepCount;
        if ~report.ResponseMetricsUsable
            report.RiseTime10to90=NaN;report.SettlingTime5Percent=NaN;
            report.OvershootPercent=NaN;report.EndWindowGain=NaN;
        end
        legend(ax,'Location','best','Box','off');
        save(fullfile(folder,[prefix 'NormalizedResponseData.mat']),'gridTime','traces','commands','report');
    else
        text(ax,0.05,0.5,'No complete command steps available for normalization.', ...
            'Units','normalized','FontSize',13);xlim(ax,[0 horizon]);ylim(ax,[0 1.5]);
    end
    grid(ax,'on');ax.GridAlpha=0.15;
    xlabel(ax,'Time after commanded step [s]');ylabel(ax,'Response / signed command step');
    title(ax,sprintf('%s | normalized observed step response',S.Name),'Interpreter','none');
    tag="INVALID";if status=="VALID",tag="VALID";end
    subtitle(ax,sprintf('%d/%d usable steps (minimum %d) | amplitude %.4g | %s',report.StepCount,report.ExpectedStepCount,report.MinimumStepCount,S.Amplitude,tag),'Interpreter','none');
    overshootText="unavailable";
    if isfinite(report.OvershootPercent),overshootText=sprintf('%.3g%%',report.OvershootPercent);end
    annotation(f,'textbox',[0.09 0.04 0.86 0.12],'EdgeColor','none', ...
        'String',sprintf('Rise 10-90%%: %s   |   Overshoot: %s   |   Settling +/-5%%: %s\nDirect pulse alignment; not PX4 system identification. Spread shows variability across pulses.', ...
        metric(report.RiseTime10to90),overshootText,metric(report.SettlingTime5Percent)), ...
        'FontSize',10,'Interpreter','none');
    style(f);
    savefig(f,fullfile(folder,[prefix 'NormalizedStepResponse.fig']));
    exportgraphics(f,fullfile(folder,[prefix 'NormalizedStepResponse.png']),'Resolution',150,'BackgroundColor','white');
    clear guard;
    f=figure('Visible','off','Color','w','Position',[100 100 1100 1000]);
    guard=onCleanup(@()close(f)); %#ok<NASGU>
    layout=tiledlayout(f,4,1,'Padding','compact','TileSpacing','compact');
    ax=nexttile(layout);plot(ax,time-D.ClockOffset,r,'Color',[0.2 0.25 0.3]);hold(ax,'on');
    plot(ax,time-D.ClockOffset,y,'Color',blue);grid(ax,'on');
    responseUnit=unit(S.Mode);
    if double(S.Mode)==4,responseUnit=responseUnit+" relative to hover target";end
    ylabel(ax,responseUnit);
    if double(S.Mode)==2&&double(S.Axis)==3
        plot(ax,D.TruthMeasuredTime-D.ClockOffset,D.TruthMeasured,'Color',[.8 .35 .15]);
        legend(ax,'Command','PX4 heading feedback','Plant yaw (absolute)','Location','best');
    else
        legend(ax,'Command','Measured','Location','best');
    end
    title(ax,sprintf('%s | %s | %s',S.Name,tag,diagnosis.Code),'Interpreter','none');
    ax=nexttile(layout);plot(ax,D.PlantTime-D.ClockOffset,D.Altitude,'Color',blue);hold(ax,'on');
    if ~isempty(estimatedTime),plot(ax,estimatedTime,estimatedHeight,'Color',[0.8 0.35 0.15]);end
    yline(ax,-double(cfg.HomeNED(3)),'--','Target','Color',[0.3 0.4 0.5]);
    yline(ax,cfg.MinTestAltitude,':','Minimum test altitude','Color',[0.4 0.5 0.6]);
    grid(ax,'on');ylabel(ax,'Altitude [m]');
    if ~isempty(estimatedTime),legend(ax,'Plant truth','PX4 estimate','Location','best');end
    ax=nexttile(layout);
    if ~isempty(verticalTime),plot(ax,verticalTime,verticalSpeed,'Color',blue);end
    hold(ax,'on');yline(ax,cfg.MaxReadyClimbRate,':','Climb gate','Color',[0.4 0.5 0.6]);
    yline(ax,-cfg.MaxReadyClimbRate,':','Color',[0.4 0.5 0.6]);grid(ax,'on');ylabel(ax,'PX4 vertical speed [m/s]');
    ax=nexttile(layout);plot(ax,D.RateTime-D.ClockOffset,D.BodyRates);grid(ax,'on');
    ylabel(ax,'Body rate [rad/s]');xlabel(ax,'Controller elapsed time [s]');
    legend(ax,'Roll','Pitch','Yaw','Location','best');
    set(findall(f,'Type','axes'),'XLim',[0 cfg.SimTime]);
    style(f);
    savefig(f,fullfile(folder,[prefix 'RawResponse.fig']));
    exportgraphics(f,fullfile(folder,[prefix 'RawResponse.png']),'Resolution',150,'BackgroundColor','white');
    clear guard;
catch ME
    report.ReportDataIssue=string(ME.message);
    if contains(ME.message,'gate did not release')
        diagnosis.Code="TAKEOFF_NOT_READY";
        diagnosis.Summary="The flight never satisfied the above-2-m readiness condition; excitation was withheld.";
        diagnosis.NextAction="Inspect takeoff command, arming, altitude, and vertical motion. Do not change rate gains for a test that never started.";
    end
end
diagnosis.ReportFile=string([prefix 'TrialReport.mat']);
diagnosis.FigureFiles=strings(0,1);
for name=["NormalizedStepResponse.fig","RawResponse.fig"]
    if isfile(fullfile(folder,string(prefix)+name))
        diagnosis.FigureFiles(end+1,1)=string(prefix)+name;
    end
end
if isempty(diagnosis.FigureFiles)
    f=figure('Visible','off','Color','w','Position',[100 100 1050 450]);
    axes(f,'Visible','off');
    text(0.03,0.8,string(S.Name)+" | "+diagnosis.Code,'FontSize',15,'Interpreter','none');
    text(0.03,0.5,diagnosis.Summary,'FontSize',10,'Interpreter','none');
    style(f);savefig(f,fullfile(folder,[prefix 'Diagnostic.fig']));close(f);
    diagnosis.FigureFiles=string([prefix 'Diagnostic.fig']);
end
% Mean-response metrics for guide-based decisions (NaN unless the trial is VALID).
diagnosis.Response=struct('OvershootPercent',field(report,'OvershootPercent'), ...
    'RiseTime10to90',field(report,'RiseTime10to90'),'SettlingTime5Percent',field(report,'SettlingTime5Percent'), ...
    'EndWindowGain',field(report,'EndWindowGain'),'ObservedPeakNormalized',field(report,'ObservedPeakNormalized'), ...
    'StepCount',report.StepCount);
report.Diagnosis=diagnosis;Report=report;
save(fullfile(folder,diagnosis.ReportFile),'Report','-v7.3');
end

function v=field(s,name)
v=NaN;if isfield(s,name)&&~isempty(s.(name)),v=double(s.(name));end
end
function [t,y]=matrix(s)
t=double(s.Time(:));a=double(s.Data);
if s.IsTimeFirst,y=reshape(a,numel(t),[]);else,y=reshape(a,[],numel(t))';end
end
function text=metric(value)
if isempty(value)||~isfinite(value),text='unavailable';else,text=sprintf('%.3g',value);end
end
function text=unit(mode)
units=["Rate [rad/s]","Angle [rad]","Velocity [m/s]","Position [m]"];
text=units(double(mode));
end
function style(f)
dark=[0.12 0.18 0.25];
set(findall(f,'Type','axes'),'Color','white','XColor',dark,'YColor',dark,'GridColor',[0.65 0.72 0.79]);
set(findall(f,'Type','text'),'Color',dark);
set(findall(f,'Type','legend'),'Color','white','TextColor',dark,'EdgeColor',[0.8 0.85 0.9]);
set(findall(f,'Type','textboxshape'),'Color',dark);
end
