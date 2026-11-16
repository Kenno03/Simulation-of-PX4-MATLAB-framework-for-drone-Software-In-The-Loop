function [control,mcmd,xerr] = lqr_hover_control(t,pos,vel,q,omega,FlightPlan,CustomLQR)
%LQR_HOVER_CONTROL Custom 12-state LQR / LQI flight controller (runs inside PX4_Receive).
%   [control,mcmd,xerr] = lqr_hover_control(t,pos,vel,q,omega,FlightPlan,CustomLQR)
%
%   Inputs (PX4 estimates, NED/FRD): t [s] model time, pos/vel [m, m/s] local position and velocity,
%   q attitude quaternion [w x y z], omega [rad/s] body rates. FlightPlan: mission reference
%   (flight_plan / flight_trajectory). CustomLQR: gains and limits (custom_controller_parameters).
%
%   Outputs: control  1x12 single, actuator_motors.control (PX4 THR_MDL_FAC = 1, so control = m.^2)
%            mcmd     4x1 rotor-speed command m in [MinCmd, 1] (the Simscape motor input)
%            xerr     12x1 state error [N E D | vN vE vD | yaw pitch roll | p q r] (logged)
%
%   Control law: m = m0 - K*xerr - Ki*zi. Position and velocity errors are rotated into the
%   yaw-aligned frame, errors are limited to the linear region (MaxPosError, MaxVelError, MaxYawError).
%   Integral action (LQI, CustomLQR.Ki nonzero): zi integrates the N, E, D and yaw errors. It starts
%   when the takeoff climb has finished, is kept in NED and rotated into the yaw frame when applied
%   (stays correct while the heading follows the path), is clamped to MaxIntegral, and does not
%   integrate while a motor command is saturated (anti-windup). Ki = 0 gives the plain LQR.
%#codegen
persistent zi tPrev
if isempty(zi), zi = zeros(4,1); tPrev = 0; end
control=single(NaN(1,12)); mcmd=zeros(4,1); xerr=zeros(12,1); td=double(t);
[pr,vrs,~,~,yr,yrate]=flight_trajectory(td,FlightPlan);
vr=zeros(3,1);
for k=1:3, if isfinite(vrs(k)), vr(k)=double(vrs(k)); end, end   % no logical indexing (code generation)
rr=double(yrate); if ~isfinite(rr), rr=0; end
% attitude from the quaternion (ZYX Euler)
w=double(q(1)); x=double(q(2)); y=double(q(3)); z=double(q(4));
roll=atan2(2*(w*x+y*z),1-2*(x*x+y*y)); s=min(max(2*(w*y-z*x),-1),1); pitch=asin(s);
yaw=atan2(2*(w*z+x*y),1-2*(y*y+z*z));
eyaw=yaw-double(yr); eyaw=atan2(sin(eyaw),cos(eyaw));
% errors in the yaw-aligned frame, limited to the linear region
c=cos(yaw); sn=sin(yaw); Rz=[c sn 0;-sn c 0;0 0 1];
epN=double(pos(:))-double(pr(:));
ep=Rz*epN; ev=Rz*(double(vel(:))-vr);
ep=min(max(ep,-CustomLQR.MaxPosError),CustomLQR.MaxPosError);
ev=min(max(ev,-CustomLQR.MaxVelError),CustomLQR.MaxVelError);
eyaw=min(max(eyaw,-CustomLQR.MaxYawError),CustomLQR.MaxYawError);
om=double(omega(:)); om(3)=om(3)-rr;
xerr=[ep;ev;eyaw;pitch;roll;om];
% integral action (in NED, applied in the yaw frame)
if td < double(FlightPlan.TakeoffStart), zi(:)=0; end
zb=[Rz*zi(1:3); zi(4)];
m=CustomLQR.m0(:)-CustomLQR.K*xerr-CustomLQR.Ki*zb;
saturated=any(m<=CustomLQR.MinCmd) || any(m>=1);
m=min(max(m,CustomLQR.MinCmd),1);
dt=min(max(td-tPrev,0),0.05); tPrev=td;
if td >= double(FlightPlan.TakeoffStart)+double(FlightPlan.TakeoffTime) && ~saturated
    eN=min(max(epN,-CustomLQR.MaxPosError),CustomLQR.MaxPosError);
    zi=zi+dt*[eN;eyaw];
    zi(1:3)=min(max(zi(1:3),-CustomLQR.MaxIntegralPos),CustomLQR.MaxIntegralPos);
    zi(4)=min(max(zi(4),-CustomLQR.MaxIntegralYaw),CustomLQR.MaxIntegralYaw);
end
if td<double(FlightPlan.TakeoffStart), m(:)=CustomLQR.IdleCmd; end
mcmd=m; control(1:4)=single(m.^2);
end
