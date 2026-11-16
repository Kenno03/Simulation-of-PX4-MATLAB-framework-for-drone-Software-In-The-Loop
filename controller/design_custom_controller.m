function D=design_custom_controller(result,options)
%DESIGN_CUSTOM_CONTROLLER Design the custom flight controller (12-state LQR and LQI) and check it.
%   D = design_custom_controller(result) designs both controllers from a hover linearisation
%   (result from linearize_drone) and checks them on the full plant with motor dynamics.
%   D = design_custom_controller(result,WriteParameters=true) also writes custom_controller_parameters.m.
%
%   Example (after Main.mlx's setup sections):
%       [~,~,~,~,result] = linearize_drone;
%       D = design_custom_controller(result);              % inspect D.Checks first
%       design_custom_controller(result,WriteParameters=true);
%
%   Design (discrete, Ts = 0.01 s, the PX4_Receive rate):
%     State x = [N E D | vN vE vD | yaw pitch roll | p q r] (linearize_drone order), input = rotor-speed
%     command per motor. Q = diag(1./xs.^2) (Bryson), xs = [0.5 m, 1 m/s, 10 deg, 30 deg/s];
%     R = 1000*I. LQR: u = m0 - K*x.
%     LQI: x is augmented with the integrals of the N, E, D and yaw errors, z(k+1) = z(k) + Ts*x_sel,
%     weighted with IntegralScale = [0.5 m*s, 0.5 rad*s]; u = m0 - K*x - Ki*z.
%   Checks (D.Checks), for 0, 1 and 2 extra samples of delay at the motors: closed-loop stability on
%   the full linearisation (propeller-angle states removed), disk phase margin at the motors, and the
%   slowest closed-loop time constant.
%
%   Options:  Ts (0.01), R (1000), StateScale ([0.5 1 deg2rad(10) deg2rad(30)]),
%             IntegralScale ([0.5 0.5]), WriteParameters (false), OutputFile (project file).
%   Requires the Control System Toolbox (dlqr, diskmargin).
arguments
    result (1,1) struct
    options.Ts (1,1) double {mustBePositive} = 0.01
    options.R (1,1) double {mustBePositive} = 1000
    options.StateScale (1,4) double {mustBePositive} = [0.5 1 deg2rad(10) deg2rad(30)]
    options.IntegralScale (1,2) double {mustBePositive} = [0.5 0.5]
    options.WriteParameters (1,1) logical = false
    options.OutputFile (1,1) string = fullfile(fileparts(mfilename('fullpath')),'custom_controller_parameters.m')
end
Ts=options.Ts;xs=repelem(options.StateScale(:),3);
s12=c2d(ss(result.Aol,result.Bol,eye(12),0),Ts,'zoh');A=s12.A;B=s12.B;
Q=diag(1./xs.^2);Rm=options.R*eye(4);
K=dlqr(A,B,Q,Rm);
Cz=zeros(4,12);Cz(1,1)=1;Cz(2,2)=1;Cz(3,3)=1;Cz(4,7)=1;     % integrated errors: N, E, D, yaw
Aa=[A zeros(12,4);Ts*Cz eye(4)];Ba=[B;zeros(4,4)];
zs=[repmat(options.IntegralScale(1),3,1);options.IntegralScale(2)];
Ka=dlqr(Aa,Ba,blkdiag(Q,diag(1./zs.^2)),Rm);Kx=Ka(:,1:12);Ki=Ka(:,13:16);
% full plant with motor dynamics; the propeller angle states are not observable/controllable
F=result.FullModel;keep=~contains(string(F.StateName),'Rz.q');
Pf=c2d(ss(F.A(keep,keep),F.B(keep,:),F.C(:,keep),0),Ts,'zoh');z=tf('z',Ts);
ctrl={ss(-K,'Ts',Ts),ss(eye(4),Ts*Cz,-Ki,-Kx,Ts)};names=["LQR","LQI"];
Checks=table();
for c=1:2
    row=table(names(c),'VariableNames',{'Design'});
    for d=0:2
        Pd=Pf;if d>0,Pd=Pf*(eye(4)*z^-d);end
        cl=feedback(ss(Pd),-ctrl{c});
        row.("Stable"+d)=isstable(cl);
        if d==0,row.SlowestTau_s=-Ts/log(max(abs(pole(cl))));end
        try,[~,mi]=diskmargin(-ctrl{c}*ss(Pd));row.("PhaseMargin"+d+"_deg")=mi.PhaseMargin(2);
        catch,row.("PhaseMargin"+d+"_deg")=NaN;end
    end
    Checks=[Checks;row]; %#ok<AGROW>
end
m0=result.u0(:);if isscalar(m0),m0=repmat(m0,4,1);end
D=struct('Ts',Ts,'K',K,'Kx',Kx,'Ki',Ki,'m0',m0,'StateScale',options.StateScale,'R',options.R, ...
    'IntegralScale',options.IntegralScale,'Checks',Checks,'ValidatedFor',designSetup());
disp(Checks);
if options.WriteParameters,writeParameters(D,options.OutputFile);end
end

function writeParameters(D,file)
% custom_controller_parameters.m with both designs; integral cap = 0.06 motor command per integrator.
capPos=0.06/max(abs(D.Ki(:,1:3)),[],'all');capYaw=0.06/max(abs(D.Ki(:,4)));
mat=@(M)strjoin(arrayfun(@(i)"    "+strjoin(compose('%.9g',M(i,:))," "),1:size(M,1)),newline);
c=D.Checks;
txt=[ ...
"function [CustomLQR, ValidatedFor] = custom_controller_parameters(design)"
"%CUSTOM_CONTROLLER_PARAMETERS Gains and limits of the custom flight controller (CUSTOM_CONTROLLER = 1)."
"%   CustomLQR = custom_controller_parameters()       LQI: 12-state LQR with integral action (default)"
"%   CustomLQR = custom_controller_parameters(""lqr"")  plain 12-state LQR (no integral action)"
"%   [CustomLQR, ValidatedFor] = ... also returns the setup the gains were designed for (total mass"
"%   [kg], inertia [kg*m^2], maximum total thrust [N]); check_gain_validity warns on another setup."
"%"
"%   Written by design_custom_controller on "+string(datetime('now','Format','yyyy-MM-dd'))+". Discrete design at Ts = "+D.Ts+" s, Bryson Q with"
sprintf("%%   xs = [%g m, %g m/s, %g deg, %g deg/s], R = %g*I. LQI integral weights %g m*s, %g rad*s.", ...
    D.StateScale(1),D.StateScale(2),rad2deg(D.StateScale(3)),rad2deg(D.StateScale(4)),D.R,D.IntegralScale(1),D.IntegralScale(2))
sprintf("%%   Checks on the full plant with motor dynamics, 0/1/2 extra samples of delay: LQR phase margin %.0f/%.0f/%.0f deg,", ...
    c.PhaseMargin0_deg(1),c.PhaseMargin1_deg(1),c.PhaseMargin2_deg(1))
sprintf("%%   LQI %.0f/%.0f/%.0f deg. Slowest time constant LQR %.2f s, LQI %.2f s. Recompute after any vehicle change.", ...
    c.PhaseMargin0_deg(2),c.PhaseMargin1_deg(2),c.PhaseMargin2_deg(2),c.SlowestTau_s(1),c.SlowestTau_s(2))
"%"
"%   State order (linearize_drone): [N E D | vN vE vD | yaw pitch roll | p q r]. Output: rotor-speed"
"%   command m per motor (Simscape input), motor order = PX4 geometry M1..M4. See lqr_hover_control."
"if nargin < 1, design = ""lqi""; end"
"CustomLQR.Ts = "+D.Ts+";"
"% Hover rotor-speed command: the linearisation point of the design."
"CustomLQR.m0 = "+mat2str(D.m0,9)+";"
"switch lower(string(design))"
"    case ""lqi"""
"        CustomLQR.K = [ ..."
mat(D.Kx)
"            ];"
"        % integral gains, columns: integrals of the N, E, D [m*s] and yaw [rad*s] errors"
"        CustomLQR.Ki = [ ..."
mat(D.Ki)
"            ];"
"    case ""lqr"""
"        CustomLQR.K = [ ..."
mat(D.K)
"            ];"
"        CustomLQR.Ki = zeros(4,4);"
"    otherwise"
"        error('custom_controller_parameters:design','Unknown design ""%s"" (use ""lqi"" or ""lqr"").',design);"
"end"
"CustomLQR.MaxPosError = 1.0;     % [m] error limit per axis (linear region)"
"CustomLQR.MaxVelError = 2.0;     % [m/s]"
"CustomLQR.MaxYawError = 0.5;     % [rad] yaw error limit (linear region)"
sprintf("CustomLQR.MaxIntegralPos = %.3g;  %% [m*s] integral limit per axis (at most 0.06 motor command)",capPos)
sprintf("CustomLQR.MaxIntegralYaw = %.3g;  %% [rad*s] yaw integral limit (at most 0.06 motor command)",capYaw)
"CustomLQR.MinCmd = 0.05;         % lowest rotor-speed command in flight"
"CustomLQR.IdleCmd = 0.2;         % armed on the ground before the takeoff reference starts"
"% Setup the design was made for (from the workspace at design time)."
sprintf("ValidatedFor = struct('Mass',%.6g,'Inertia',[%.6g %.6g %.6g],'MaxThrust',%.6g);", ...
    D.ValidatedFor.Mass,D.ValidatedFor.Inertia,D.ValidatedFor.MaxThrust)
"end"];
fid=fopen(file,'w');fprintf(fid,'%s\n',txt);fclose(fid);
fprintf('Wrote %s\n',file);
end

function V=designSetup()
% Mass, inertia and maximum thrust of the setup in the base workspace (the one linearised); NaN if unknown.
V=struct('Mass',NaN,'Inertia',[NaN NaN NaN],'MaxThrust',NaN);
try
    SP=evalin('base','SimParams');J=SP.Inertia;if ~isvector(J),J=diag(J);end
    V=struct('Mass',SP.TotalMass,'Inertia',J(:)','MaxThrust',SP.CommandMaxTotalThrust);
catch
end
end
