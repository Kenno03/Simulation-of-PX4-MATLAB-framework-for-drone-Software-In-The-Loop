function Result = compute_propeller_coefficients(Propeller)
%COMPUTE_PROPELLER_COEFFICIENTS
% Compute static rotor/propeller CT and CQ using BEMT.
%
% Output:
%   Result.CT          Rotor thrust coefficient
%   Result.CQ          Rotor torque coefficient
%   Result.kT_omega    T = kT_omega * omega^2
%   Result.kQ_omega    Q = kQ_omega * omega^2

%% Input data

B = Propeller.BEMT;

Nb = B.NumberOfBlades;
R  = Propeller.Radius;
c  = B.Chord;

clalpha = B.LiftCurveSlope;
cd0     = B.MeanDragCoefficient;

rho = B.ReferenceDensity;

N = B.NumberOfElements;

%% Check required information

if ~isfinite(c) || c <= 0
    error("BEMT calculation requires a valid mean blade chord.");
end

if ~isfinite(clalpha) || clalpha <= 0
    error("BEMT calculation requires a valid lift curve slope.");
end

if ~isfinite(cd0) || cd0 < 0
    error("BEMT calculation requires a valid mean drag coefficient.");
end

%% Rotor solidity

sigma = Nb*c/(pi*R);

%% Radial discretization

rEdges = linspace(1e-3, 1, N + 1);

dr = diff(rEdges);

rMid = ...
    rEdges(1:end-1) + dr/2;

%% Initialize

CT = 0;
CQ_induced = 0;

%% Integrate along blade

for k = 1:N

    r = rMid(k);

    %% Blade angle

    switch B.TwistType

        case "Ideal"

            theta = B.ThetaTip/r;

        case "Linear"

            theta = ...
                B.ThetaRoot + ...
                B.ThetaTwist*r;

        otherwise

            error( ...
                "Unknown BEMT twist type: %s", ...
                B.TwistType);
    end

    if theta <= 0
        error( ...
            "Blade pitch became non-positive at r = %.3f.", ...
            r);
    end

    %% Solve BEMT inflow equation

    residual = @(lambda) ...
        4*tipLossFactor(r,lambda,Nb)*lambda.^2 ...
        - 0.5*sigma*clalpha*(theta*r-lambda);

    lambdaMin = 1e-10;
    lambdaMax = theta*r;

    lambda = fzero( ...
        residual, ...
        [lambdaMin lambdaMax]);

    %% Incremental thrust coefficient

    F = tipLossFactor(r,lambda,Nb);

    dCT = ...
        4*F*lambda^2*r*dr(k);

    CT = CT + dCT;

    %% Induced torque contribution

    CQ_induced = ...
        CQ_induced + lambda*dCT;

end

%% Profile drag contribution

CQ_profile = ...
    sigma*cd0/8;

CQ = ...
    CQ_induced + CQ_profile;

%% Dimensional omega-based coefficients

kT_omega = ...
    CT*rho*pi*R^4;

kQ_omega = ...
    CQ*rho*pi*R^5;

%% Standard dimensionless propeller coefficients

KT_prop = ...
    0.25*pi^3*CT;

KQ_prop = ...
    0.125*pi^3*CQ;

%% Output structure

Result = struct();

Result.PropellerID   = Propeller.ID;
Result.PropellerName = Propeller.Name;

Result.Source = "BEMT";
Result.Date   = datetime("now");

Result.CT = CT;
Result.CQ = CQ;

Result.CQ_Induced = CQ_induced;
Result.CQ_Profile = CQ_profile;

Result.Propeller_KT = KT_prop;
Result.Propeller_KQ = KQ_prop;

Result.kT_omega = kT_omega;
Result.kQ_omega = kQ_omega;

Result.ReferenceDensity = rho;
Result.Solidity = sigma;

%% Display

fprintf("\n");
fprintf("========================================\n");
fprintf(" Propeller BEMT Results\n");
fprintf("========================================\n");

fprintf(" Propeller       : %s\n", ...
    Propeller.Name);

fprintf(" CT              : %.8f\n", ...
    CT);

fprintf(" CQ              : %.8f\n", ...
    CQ);

fprintf(" kT [omega form] : %.6e N*s^2/rad^2\n", ...
    kT_omega);

fprintf(" kQ [omega form] : %.6e N*m*s^2/rad^2\n", ...
    kQ_omega);

fprintf("========================================\n");

%% Save BEMT result

propellerFolder = ...
    fullfile(pwd,"Propeller");

if ~isfolder(propellerFolder)
    mkdir(propellerFolder);
end

fileName = ...
    Propeller.ID + "_BEMT.mat";

filePath = ...
    fullfile(propellerFolder,fileName);

save(filePath,"Result");

fprintf("\nBEMT result saved:\n");
fprintf(" %s\n\n",filePath);

end


%% Local function

function F = tipLossFactor(r,lambda,Nb)

phi = lambda/r;

phi = max(phi,1e-12);

f = ...
    0.5*Nb*(1-r)/(r*phi);

F = ...
    (2/pi)*acos(exp(-f));

% Avoid numerical zero near blade tip
F = max(F,1e-8);

end