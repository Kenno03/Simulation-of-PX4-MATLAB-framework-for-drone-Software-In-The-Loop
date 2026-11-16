function M=score_mission(plant,plan,offset,simTime)
%SCORE_MISSION Path-tracking metrics of one mission flight, from the plant's physical truth.
%   M = score_mission(plantData,plan,offset,simTime) scores the flight in the plant SDI dataset
%   (Simscape truth, not PX4's estimate) against the reference of FlightPlan PLAN (flight_trajectory).
%   OFFSET maps plant time to mission time (controller_clock_offset); SIMTIME is the flight length.
%
%   Scored window: the path itself, from the end of takeoff + settling to the end of the lap.
%   M.Summary (one row, lower is better):
%     RMS3D_cm        3-D position error against the moving reference, RMS
%     MaxXY_cm        largest horizontal error
%     PathDev_RMS_cm  distance to the reference PATH (timing-independent), RMS; PathDev_Max_cm max
%     Lag_s           delay that best aligns the flown path with the reference
%     AltRMS_cm, AltMax_cm        altitude error
%     HeadingRMS_deg, HeadingMax_deg   heading error (truth vs reference heading)
%     Settle_s        time after the lap until the horizontal error stays below 5 cm
%     MaxTilt_deg     largest roll/pitch while flying (takeoff end to simTime - 1 s)
%     MotorRate_rms   RMS rate of change of the motor commands on the path [1/s] (smoothness)
%     MaxMotor        largest motor command while flying
%     Score           composite: RMS3D + 0.5*MaxXY + PathDev + 0.5*AltRMS [cm] + HeadingRMS [deg]
%                     + 2*Lag [s] + min(Settle, 30) [s]. The weights are a stated choice.
%   M also holds the time series used for plots: t (mission time), p, ref (North, East, altitude),
%   eta (roll, pitch, yaw [deg]), yref, eyaw, motor, window and the reference path.
state=plant.getElement('Drone State').Values;
[t,p]=readts(state.S_NED);p(:,3)=-p(:,3);
[~,eta]=readts(state.Eta_NED);eta=rad2deg(eta(:,[3 2 1]));   % roll, pitch, yaw [deg]
tm=t-offset;n=numel(t);
ref=zeros(n,3);yref=zeros(n,1);
for k=1:n,[r,~,~,~,yr]=flight_trajectory(tm(k),plan);ref(k,:)=double(r);yref(k)=double(yr);end
ref(:,3)=-ref(:,3);
motor=zeros(n,4);
for k=1:4,[tk,u]=readts(plant.getElement(sprintf('Motor %d',k)).Values);motor(:,k)=interp1(tk,u,t,'previous','extrap');end
tMove=plan.TakeoffStart+plan.TakeoffTime+plan.SettleTime;tEnd=tMove+4*(plan.LegTime+plan.DwellTime);
win=tm>=tMove&tm<=tEnd;post=tm>tEnd&tm<=simTime-1;
flying=tm>=plan.TakeoffStart+plan.TakeoffTime&tm<=simTime-1;   % excludes the shutdown (PX4 stopped, plant falling)
e=p-ref;exy=vecnorm(e(:,1:2),2,2);
% distance to the reference path itself (timing-independent)
tp=(tMove:0.005:tEnd)';path=zeros(numel(tp),2);
for k=1:numel(tp),r=double(flight_trajectory(tp(k),plan));path(k,:)=r(1:2);end
pw=p(win,1:2);dev=zeros(size(pw,1),1);
for k=1:size(pw,1),dev(k)=sqrt(min(sum((path-pw(k,:)).^2,2)));end
% lag: delay that best aligns the flown path with the reference
taus=0:0.02:2;J=zeros(size(taus));tw=tm(win);
for j=1:numel(taus)
    rj=zeros(numel(tw),2);
    for k=1:numel(tw),r=double(flight_trajectory(tw(k)-taus(j),plan));rj(k,:)=r(1:2);end
    J(j)=sqrt(mean(sum((pw-rj).^2,2)));
end
[~,jb]=min(J);lag=taus(jb);
eyaw=rad2deg(atan2(sind(eta(:,3)-rad2deg(yref)),cosd(eta(:,3)-rad2deg(yref))));
idx=find(post&exy>0.05,1,'last');
if isempty(idx),settle=0;elseif idx==find(post,1,'last'),settle=Inf;else,settle=tm(idx)-tEnd;end
du=diff(motor)./diff(t);
S=table();
S.RMS3D_cm=100*sqrt(mean(sum(e(win,:).^2,2)));
S.MaxXY_cm=100*max(exy(win));
S.PathDev_RMS_cm=100*sqrt(mean(dev.^2));
S.PathDev_Max_cm=100*max(dev);
S.Lag_s=lag;
S.AltRMS_cm=100*sqrt(mean(e(win,3).^2));
S.AltMax_cm=100*max(abs(e(win,3)));
S.HeadingRMS_deg=sqrt(mean(eyaw(win).^2));
S.HeadingMax_deg=max(abs(eyaw(win)));
S.Settle_s=settle;
S.MaxTilt_deg=max(max(abs(eta(flying,1:2))));
S.MotorRate_rms=sqrt(mean(du(win(1:end-1),:).^2,'all'));
S.MaxMotor=max(motor(flying,:),[],'all');
S.Score=S.RMS3D_cm+0.5*S.MaxXY_cm+S.PathDev_RMS_cm+0.5*S.AltRMS_cm+S.HeadingRMS_deg+2*S.Lag_s+min(S.Settle_s,30);
M=struct('Summary',S,'t',tm,'p',p,'ref',ref,'eta',eta,'yref',rad2deg(yref),'eyaw',eyaw,'motor',motor, ...
    'window',[tMove tEnd],'path',path);
end

function [t,y]=readts(s)
t=double(s.Time(:));a=double(s.Data);
if s.IsTimeFirst,y=reshape(a,numel(t),[]);else,y=reshape(a,[],numel(t))';end
end
