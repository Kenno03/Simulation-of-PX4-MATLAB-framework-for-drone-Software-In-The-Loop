function [position,velocity,acceleration,jerk,yaw,yawspeed] = flight_trajectory(t,Plan)
%FLIGHT_TRAJECTORY Smooth square, circle or figure-8 (PX4 NED, metres, seconds).
%#codegen
% Options (flight_plan defaults keep the previous behaviour):
%   Plan.VelocityFeedforward  true: also return the analytic reference velocity (and yaw rate);
%                             false: velocity/yawspeed are NaN (position setpoints only).
%   Plan.HeadingAlongPath     true: circle/figure-8 yaw follows the path tangent; the vehicle turns to
%                             the start heading during the settle time. false: fixed North heading.
td=double(t);
x=0.0; y=0.0; vx=0.0; vy=0.0; psi=0.0; dpsi=0.0;
[s,ds]=smooth_step((td-Plan.TakeoffStart)/Plan.TakeoffTime);
z=-Plan.Altitude*s; vz=-Plan.Altitude*ds/Plan.TakeoffTime;
tUp=Plan.TakeoffStart+Plan.TakeoffTime;
tMove=tUp+Plan.SettleTime;
leg=Plan.LegTime; stride=leg+Plan.DwellTime;
if Plan.Kind == uint8(1)
    % Preserve the existing square's four independent straight legs.
    [s1,d1]=smooth_step((td-tMove)/leg);[s3,d3]=smooth_step((td-tMove-2*stride)/leg);
    [s2,d2]=smooth_step((td-tMove-stride)/leg);[s4,d4]=smooth_step((td-tMove-3*stride)/leg);
    x=Plan.Size*(s1-s3); vx=Plan.Size*(d1-d3)/leg;
    y=Plan.Size*(s2-s4); vy=Plan.Size*(d2-d4)/leg;
elseif Plan.Kind == uint8(2) || Plan.Kind == uint8(3)
    [s,ds]=smooth_step((td-tMove)/(4*stride));
    phase=2*pi*s; dphase=2*pi*ds/(4*stride);
    if Plan.Kind == uint8(2)
        r=Plan.Size/2;
        x=r*(1-cos(phase)); y=r*sin(phase);
        A=sin(phase); B=cos(phase); dA=cos(phase); dB=-sin(phase);    % tangent d/dphase (scaled)
        vx=r*A*dphase; vy=r*B*dphase;
    else
        % Figure-8 (lemniscate of Gerono) through the start point: both turn directions in one lap.
        a=Plan.Size/2;
        x=a*sin(phase); y=a/2*sin(2*phase);
        A=cos(phase); B=cos(2*phase); dA=-sin(phase); dB=-2*sin(2*phase);
        vx=a*A*dphase; vy=a*B*dphase;
    end
    if Plan.HeadingAlongPath
        psiPath=atan2(B,A);                                   % direction of motion (North = 0, East = +pi/2)
        dpsiPath=(A*dB-B*dA)/(A*A+B*B);                       % d(heading)/d(phase)
        if td<tMove
            [sy,dsy]=smooth_step((td-tUp)/Plan.SettleTime);   % turn to the start heading while settling
            psi=psiPath*sy; dpsi=psiPath*dsy/Plan.SettleTime;
        else
            psi=psiPath; dpsi=dpsiPath*dphase;
        end
        psi=atan2(sin(psi),cos(psi));
    end
end
position=single([x,y,z]);
yaw=single(psi);
if Plan.VelocityFeedforward
    velocity=single([vx,vy,vz]);
    yawspeed=single(dpsi);
else
    velocity=single([NaN,NaN,NaN]);
    yawspeed=single(NaN);
end
acceleration=single([NaN,NaN,NaN]);
jerk=single([NaN,NaN,NaN]);
end

function [s,ds]=smooth_step(t)
inside=t>0&&t<1;
t=min(max(t,0),1);
s=3*t*t-2*t*t*t;
ds=0;if inside,ds=6*t-6*t*t;end
end
