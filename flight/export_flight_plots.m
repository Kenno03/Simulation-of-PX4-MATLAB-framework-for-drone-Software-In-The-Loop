function metrics=export_flight_plots(plant,folder,plan,offset)
%EXPORT_FLIGHT_PLOTS Detailed plots of one mission flight from the plant truth (FIG and PNG).
%   metrics = export_flight_plots(plantData,folder,plan,offset) writes to FOLDER: 3-D path, top view,
%   position, position error, attitude, body rates, motor commands, rotor speeds, total thrust and every
%   logged plant state, plus plot_data.mat and metrics.json. OFFSET maps plant time to mission time
%   (controller_clock_offset). run_mission calls it for every flight; score_mission gives the scores.
% offset converts plant time to controller mission time; record its provenance.
if ~isfolder(folder),mkdir(folder);end
state=plant.getElement('Drone State').Values;
[t,p]=readts(state.S_NED);p(:,3)=-p(:,3);
[~,eta]=readts(state.Eta_NED);eta=eta(:,[3 2 1])*180/pi;
% Omega_frd is not exported as a top-level SDI element in every project
% configuration. Prefer the direct element, then look for a nested logged
% element or the same signal inside the Drone State bus.
rateSignal=findLoggedSignal(plant,'Omega_frd');
if isempty(rateSignal) && isstruct(state)
    if isfield(state,'Omega_frd')
        rateSignal=state.Omega_frd;
    elseif isfield(state,'Omega_NED')
        % Drone_simulation_PX4 exposes this bus field as Omega_NED, although
        % the source signal propagated into the subsystem is Omega_frd.
        rateSignal=state.Omega_NED;
    end
end
if isempty(rateSignal)
    warning('export_flight_plots:MissingOmegaFRD', ...
        ['Omega_frd was not present in this plant SDI run. The body-rate ' ...
         'plot will be skipped; position, attitude and actuator plots can still be exported.']);
    tr=[];rates=[];
else
    [tr,rates]=readts(rateSignal);
end
reference=zeros(numel(t),3);
for k=1:numel(t),reference(k,:)=double(flight_trajectory(t(k)-offset,plan));end
reference(:,3)=-reference(:,3);
motor=zeros(numel(t),4);rpm=motor;
for k=1:4
 [tm,u]=readts(plant.getElement(sprintf('Motor %d',k)).Values);
 motor(:,k)=interp1(tm,u,t,'previous','extrap');
 [tw,w]=readts(plant.getElement(sprintf('w%d',k)).Values);
 rpm(:,k)=abs(interp1(tw,w,t,'linear','extrap'))*60/(2*pi);
end
[tf,thrust]=readts(plant.getElement('Total Thrust').Values);
active=t>=offset+plan.TakeoffStart+plan.TakeoffTime;
tMove=plan.TakeoffStart+plan.TakeoffTime+plan.SettleTime;
moving=t>=offset+tMove & t<=offset+tMove+4*(plan.LegTime+plan.DwellTime);   % the path itself
error=p-reference;
metrics=struct('mission',char(string(plan.Kind)),'clockOffset',offset, ...
 'trackingRMSE',sqrt(mean(error(moving,:).^2,1)), ...
 'maximumRollPitchDeg',max(abs(eta(active,1:2)),[],1), ...
 'maxMotorCommand',max(motor(active,:),[],'all'), ...
 'highSaturationFraction',mean(any(motor(active,:)>=0.99,2)), ...
 'finalNorthEastAltitude',p(end,:),'rawTimeEnd',t(end));
fid=fopen(fullfile(folder,'metrics.json'),'w');fprintf(fid,'%s',jsonencode(metrics,PrettyPrint=true));fclose(fid);
save(fullfile(folder,'plot_data.mat'),'t','p','reference','eta','tr','rates','motor','rpm','tf','thrust','plan','offset','metrics');
q=1:10:numel(t);
f=figure('Visible','off','Color','w','Position',[100 100 1050 700]);
plot3(reference(q,1),reference(q,2),reference(q,3),'--','LineWidth',1.5);hold on;
plot3(p(q,1),p(q,2),p(q,3),'LineWidth',1.3);grid on;axis equal;view(45,28);
xlabel('North [m]');ylabel('East [m]');zlabel('Altitude [m]');legend('Command','Simscape truth','Location','best');
title('Flight path in North, East and altitude');output(f,folder,'01_xyz');
f=figure('Visible','off','Color','w');plot(reference(q,2),reference(q,1),'--',p(q,2),p(q,1),'-');axis equal;grid on;
xlabel('East [m]');ylabel('North [m]');legend('Command','Simscape truth');title('Top view');output(f,folder,'02_top_view');
labels={'North','East','Altitude'};
f=figure('Visible','off','Color','w','Position',[100 100 1000 800]);
for k=1:3,subplot(3,1,k);plot(t(q)-offset,reference(q,k),'--',t(q)-offset,p(q,k));grid on;ylabel([labels{k} ' [m]']);end
xlabel('Mission time [s]');legend('Command','Simscape truth');output(f,folder,'03_position');
f=figure('Visible','off','Color','w');plot(t(q)-offset,error(q,:));grid on;xlabel('Mission time [s]');ylabel('Position error [m]');legend(labels);output(f,folder,'04_position_error');
f=figure('Visible','off','Color','w');plot(t(q)-offset,eta(q,:));grid on;xlabel('Mission time [s]');ylabel('Angle [deg]');legend('Roll','Pitch','Yaw');output(f,folder,'05_attitude');
if ~isempty(tr)
 f=figure('Visible','off','Color','w');plot(tr(1:10:end)-offset,rates(1:10:end,:));grid on;xlabel('Mission time [s]');ylabel('Body angular velocity [rad/s]');legend('p','q','r');output(f,folder,'06_body_rates');
end
f=figure('Visible','off','Color','w');plot(t(q)-offset,motor(q,:));grid on;xlabel('Mission time [s]');ylabel('Motor command [speed fraction]');ylim([0 1.05]);legend('M1','M2','M3','M4');output(f,folder,'07_motor_commands');
f=figure('Visible','off','Color','w');plot(t(q)-offset,rpm(q,:));grid on;xlabel('Mission time [s]');ylabel('Rotor speed [RPM]');legend('M1','M2','M3','M4');output(f,folder,'08_rotor_speeds');
f=figure('Visible','off','Color','w');plot(tf(1:10:end)-offset,thrust(1:10:end));grid on;xlabel('Mission time [s]');ylabel('Total thrust [N]');output(f,folder,'09_thrust');
% Export every logged physical state as individual component plots as well.
fields=fieldnames(state);
for j=1:numel(fields)
 [tx,y]=readts(state.(fields{j}));
 f=figure('Visible','off','Color','w');plot(tx(1:10:end)-offset,y(1:10:end,:));grid on;
 xlabel('Mission time [s]');title(strrep(fields{j},'_',' '));output(f,folder,['state_' fields{j}]);
end
disp(metrics);
end
function [t,y]=readts(s)
t=double(s.Time(:));a=double(s.Data);
if s.IsTimeFirst,y=reshape(a,numel(t),[]);else,y=reshape(a,[],numel(t))';end
end
function signal=findLoggedSignal(dataset,name)
signal=[];
names=dataset.getElementNames;
if isempty(names),return;end
idx=find(strcmp(names,name),1);
if isempty(idx)
    nested=endsWith(names,['/' name]) | endsWith(names,['.' name]);
    idx=find(nested,1,'last');
end
if ~isempty(idx)
    element=dataset.getElement(idx);
    signal=element.Values;
end
end
function output(f,folder,name)
savefig(f,fullfile(folder,[name '.fig']));
exportgraphics(f,fullfile(folder,[name '.png']),'Resolution',130);close(f);
end
