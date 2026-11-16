function P = compute_drone_geometry_inertia(P, mass_mode)
%COMPUTE_DRONE_GEOMETRY_INERTIA
% Finalize geometry, mass, COM, inertia and rotor locations for an
% X-configuration quadrotor.
%
% Body coordinate convention:
%
%   +x = forward
%   +y = right
%   +z = down
%
% Main purpose:
%   - calculate mass properties from simplified component geometry
%   - optionally use user-defined overrides
%   - optionally import CAD-derived mass properties
%   - always populate P.Derived.*
%
% Supported data sources, in priority order:
%
%   1. P.User.* overrides
%   2. P.CAD.* data converted to P.User.* overrides
%   3. Simplified geometry-based estimate
%
% Required fields:
%
%   P.Body_Radius
%   P.Body_Height
%   P.Body_Mass
%
%   P.Arm_Length
%   P.Arm_Radius
%   P.Arm_Mass
%
%   P.Motor_Radius
%   P.Motor_Height
%   P.Motor_Mass
%
%   P.Propeller_Mass
%
% Optional user overrides:
%
%   P.User.COM
%   P.User.GeomCenter
%   P.User.Inertia_COM
%   P.User.Total_Mass
%   P.User.Rotor_Positions_Origin
%
% Optional CAD fields:
%
%   P.CAD.Origin_Global_mm
%   P.CAD.COM_Global_mm
%   P.CAD.PrincipalMoments_gmm2
%   P.CAD.PrincipalAxes_CAD
%   P.CAD.R_body_from_cad
%   P.CAD.Rotor_Positions_Global_mm
%
% mass_mode:
%
%   "body_plus_parts"
%       Total mass = body + arms + motors + propellers
%
%   "total_minus_parts"
%       Body mass = specified total mass - known parts
%
% Results are stored in:
%
%   P.Derived.*


%% ------------------------------------------------------------------------
% Default mass mode
% -------------------------------------------------------------------------

if nargin < 2 || isempty(mass_mode)

    mass_mode = "body_plus_parts";

end


notice = @(msg) fprintf("[NOTICE] %s\n", msg);


%% ------------------------------------------------------------------------
% Ensure optional structures exist
% -------------------------------------------------------------------------

if ~isfield(P,'User') || isempty(P.User)

    P.User = struct();

end


if ~isfield(P,'CAD') || isempty(P.CAD)

    P.CAD = struct();

end


%% ------------------------------------------------------------------------
% Check required fields
% -------------------------------------------------------------------------

requiredFields = [ ...
    "Body_Radius"
    "Body_Height"
    "Body_Mass"
    "Arm_Length"
    "Arm_Radius"
    "Arm_Mass"
    "Motor_Radius"
    "Motor_Height"
    "Motor_Mass"
    "Propeller_Mass"
];


for k = 1:numel(requiredFields)

    if ~isfield(P, requiredFields(k))

        error( ...
            "Missing required field P.%s", ...
            requiredFields(k));

    end

end


%% ------------------------------------------------------------------------
% Basic parameter validation
% -------------------------------------------------------------------------

if P.Body_Radius <= 0
    error("P.Body_Radius must be positive.");
end

if P.Body_Height <= 0
    error("P.Body_Height must be positive.");
end

if P.Body_Mass <= 0
    error("P.Body_Mass must be positive.");
end

if P.Arm_Length <= 0
    error("P.Arm_Length must be positive.");
end

if P.Arm_Radius <= 0
    error("P.Arm_Radius must be positive.");
end

if P.Arm_Mass < 0
    error("P.Arm_Mass cannot be negative.");
end

if P.Motor_Radius <= 0
    error("P.Motor_Radius must be positive.");
end

if P.Motor_Height <= 0
    error("P.Motor_Height must be positive.");
end

if P.Motor_Mass < 0
    error("P.Motor_Mass cannot be negative.");
end

if P.Propeller_Mass < 0
    error("P.Propeller_Mass cannot be negative.");
end


%% ------------------------------------------------------------------------
% Simplified geometry offsets
%
% Coordinate convention:
%
%   +z = down
%
% Therefore:
%
%   body top    = -Body_Height/2
%   body bottom = +Body_Height/2
%
% The arms are located ABOVE the bottom surface, therefore their z
% coordinate is reduced.
%
% The motor axis is located ABOVE the arm axis, so its z coordinate is
% reduced again.
% -------------------------------------------------------------------------

arm_height_above_bottom = ...
    0.02;                                   % [m]

motor_axis_above_arm = ...
    P.Arm_Radius + P.Motor_Radius;          % [m]

include_prop_pointmass = ...
    true;


%% ------------------------------------------------------------------------
% Geometry helpers
% -------------------------------------------------------------------------

% Solid cylinder inertia about its own COM.
%
% Cylinder axis is local z.

cyl_inertia_com = @(m,R,H) ...
    diag([ ...
        (1/12)*m*(3*R^2 + H^2), ...
        (1/12)*m*(3*R^2 + H^2), ...
        (1/2)*m*R^2 ...
    ]);


% Parallel-axis theorem

parax = @(Icom,m,r) ...
    Icom + ...
    m * ...
    ( ...
        dot(r,r)*eye(3) ...
        - r(:)*r(:).' ...
    );


%% ------------------------------------------------------------------------
% Optional CAD transformation
% -------------------------------------------------------------------------

if isfield(P.CAD,'R_body_from_cad') && ...
        ~isempty(P.CAD.R_body_from_cad)

    R_body_from_cad = ...
        P.CAD.R_body_from_cad;

else

    R_body_from_cad = ...
        eye(3);

end


if ~isequal(size(R_body_from_cad), [3 3])

    error( ...
        "P.CAD.R_body_from_cad must be 3x3.");

end


%% ------------------------------------------------------------------------
% Import COM from CAD
% -------------------------------------------------------------------------

if isfield(P.CAD,'Origin_Global_mm') && ...
   isfield(P.CAD,'COM_Global_mm') && ...
   (~isfield(P.User,'COM') || isempty(P.User.COM))

    Omm = ...
        P.CAD.Origin_Global_mm(:);

    Gmm = ...
        P.CAD.COM_Global_mm(:);


    if numel(Omm) ~= 3 || numel(Gmm) ~= 3

        error( ...
            "P.CAD.Origin_Global_mm and " + ...
            "P.CAD.COM_Global_mm must be 3-element vectors.");

    end


    % CAD-frame displacement from model origin to COM

    r_GO_cad_m = ...
        (Gmm - Omm) * 1e-3;


    % Convert to body frame

    r_GO_body_m = ...
        R_body_from_cad * ...
        r_GO_cad_m;


    P.User.COM = ...
        r_GO_body_m.';


    notice( ...
        "Loaded COM override from P.CAD.");

end


%% ------------------------------------------------------------------------
% Import inertia matrix from CAD
% -------------------------------------------------------------------------

if isfield(P.CAD,'PrincipalMoments_gmm2') && ...
   isfield(P.CAD,'PrincipalAxes_CAD') && ...
   (~isfield(P.User,'Inertia_COM') || isempty(P.User.Inertia_COM))

    % g*mm^2 -> kg*m^2

    lambda = ...
        P.CAD.PrincipalMoments_gmm2(:) * 1e-9;

    Vraw = ...
        P.CAD.PrincipalAxes_CAD;


    if numel(lambda) ~= 3

        error( ...
            "P.CAD.PrincipalMoments_gmm2 must contain 3 values.");

    end


    if ~isequal(size(Vraw), [3 3])

        error( ...
            "P.CAD.PrincipalAxes_CAD must be 3x3.");

    end


    % CAD principal axes may contain small numerical errors.
    % Orthogonalize them before forming the inertia tensor.

    [U,~,V] = ...
        svd(Vraw,'econ');

    Vcad = ...
        U*V';


    % Convert principal axes to body coordinates

    Vbody = ...
        R_body_from_cad * ...
        Vcad;


    % Full inertia tensor

    Jcad = ...
        Vbody * ...
        diag(lambda) * ...
        Vbody.';


    % Enforce symmetry

    Jcad = ...
        0.5 * ...
        (Jcad + Jcad.');


    P.User.Inertia_COM = ...
        Jcad;


    notice( ...
        "Loaded FULL inertia matrix override from P.CAD.");

end


%% ------------------------------------------------------------------------
% Import exact rotor positions from CAD
% -------------------------------------------------------------------------

if isfield(P.CAD,'Rotor_Positions_Global_mm') && ...
   isfield(P.CAD,'Origin_Global_mm') && ...
   (~isfield(P.User,'Rotor_Positions_Origin') || ...
    isempty(P.User.Rotor_Positions_Origin))

    Rg = ...
        P.CAD.Rotor_Positions_Global_mm;


    if isequal(size(Rg), [3 4])

        Rg = ...
            Rg.';

    end


    if ~isequal(size(Rg), [4 3])

        error( ...
            "P.CAD.Rotor_Positions_Global_mm " + ...
            "must be 4x3 or 3x4.");

    end


    Omm = ...
        P.CAD.Origin_Global_mm(:).';


    rotor_rel_cad_m = ...
        (Rg - Omm) * 1e-3;


    rotor_rel_body_m = ...
        ( ...
            R_body_from_cad * ...
            rotor_rel_cad_m.' ...
        ).';


    P.User.Rotor_Positions_Origin = ...
        rotor_rel_body_m;


    notice( ...
        "Loaded rotor position override from P.CAD.");

end


%% ------------------------------------------------------------------------
% Geometric center
% -------------------------------------------------------------------------

r_geom = ...
    [0; 0; 0];


if isfield(P.User,'GeomCenter') && ...
        ~isempty(P.User.GeomCenter)

    r_geom = ...
        P.User.GeomCenter(:);


    if numel(r_geom) ~= 3

        error( ...
            "P.User.GeomCenter must be a 3-element vector.");

    end


    notice( ...
        "Using user-provided geometric center P.User.GeomCenter.");

else

    notice( ...
        "Geometric center assumed at [0 0 0] (model origin).");

end


%% ------------------------------------------------------------------------
% Optional user total-mass override
% -------------------------------------------------------------------------

if isfield(P.User,'Total_Mass') && ...
        ~isempty(P.User.Total_Mass)

    P.Total_Mass = ...
        P.User.Total_Mass;


    notice( ...
        "Using user-provided P.User.Total_Mass as P.Total_Mass.");

end


%% ------------------------------------------------------------------------
% Component mass bookkeeping
% -------------------------------------------------------------------------

m_arms = ...
    4 * P.Arm_Mass;

m_motors = ...
    4 * P.Motor_Mass;

m_props = ...
    4 * P.Propeller_Mass;

m_known = ...
    m_arms + ...
    m_motors + ...
    m_props;


%% ------------------------------------------------------------------------
% Determine body / total mass
% -------------------------------------------------------------------------

switch string(mass_mode)

    case "body_plus_parts"

        P.Total_Mass = ...
            P.Body_Mass + ...
            m_known;


        notice( ...
            "Estimated P.Total_Mass = P.Body_Mass + known parts " + ...
            "(mass_mode='body_plus_parts').");


    case "total_minus_parts"

        if ~isfield(P,'Total_Mass') || ...
                isempty(P.Total_Mass)

            error( ...
                "mass_mode='total_minus_parts' requires " + ...
                "P.Total_Mass to be set.");

        end


        P.Body_Mass = ...
            P.Total_Mass - ...
            m_known;


        notice( ...
            "Estimated P.Body_Mass = P.Total_Mass - known parts " + ...
            "(mass_mode='total_minus_parts').");


    otherwise

        error( ...
            "Unknown mass_mode '%s'. " + ...
            "Use 'body_plus_parts' or 'total_minus_parts'.", ...
            mass_mode);

end


if P.Body_Mass <= 0

    error( ...
        "Computed Body_Mass <= 0. Check masses and mass_mode.");

end


%% ------------------------------------------------------------------------
% Fallback z geometry
%
% IMPORTANT:
%
% +z is DOWN.
%
% Therefore the bottom of the body is positive z.
% -------------------------------------------------------------------------

z_bottom = ...
    +P.Body_Height/2;


% Arm lies above the bottom surface

z_arm = ...
    z_bottom - ...
    arm_height_above_bottom;


% Motor / rotor axis lies above the arm

z_motor = ...
    z_arm - ...
    motor_axis_above_arm;


z_prop = ...
    z_motor;


%% ------------------------------------------------------------------------
% Rotor positions
% -------------------------------------------------------------------------

use_rotor_override = ...
    isfield(P.User,'Rotor_Positions_Origin') && ...
    ~isempty(P.User.Rotor_Positions_Origin);


if use_rotor_override

    rotor_pos_origin = ...
        P.User.Rotor_Positions_Origin;


    if isequal(size(rotor_pos_origin), [3 4])

        rotor_pos_origin = ...
            rotor_pos_origin.';

    end


    if ~isequal(size(rotor_pos_origin), [4 3])

        error( ...
            "P.User.Rotor_Positions_Origin must be 4x3 or 3x4.");

    end


    notice( ...
        "Using user-provided rotor positions.");


    L_each = ...
        vecnorm( ...
            rotor_pos_origin(:,1:2), ...
            2, ...
            2);


    if any(L_each < 1e-9)

        error( ...
            "Rotor xy-position too small. " + ...
            "Cannot determine arm direction.");

    end

else

    rotor_pos_origin = ...
        zeros(4,3);

    L_each = ...
        P.Arm_Length * ...
        ones(4,1);

end


%% ------------------------------------------------------------------------
% Component structure
%
% Each component contains:
%
%   name
%   mass
%   COM position
%   inertia about own COM
% -------------------------------------------------------------------------

components = ...
    struct( ...
        'name', {}, ...
        'm',    {}, ...
        'r',    {}, ...
        'Icom', {} );


%% ------------------------------------------------------------------------
% Main body
% -------------------------------------------------------------------------

components(end+1).name = ...
    "body";

components(end).m = ...
    P.Body_Mass;

components(end).r = ...
    [0;0;0];

components(end).Icom = ...
    cyl_inertia_com( ...
        P.Body_Mass, ...
        P.Body_Radius, ...
        P.Body_Height);


%% ------------------------------------------------------------------------
% X-configuration directions
%
% Vehicle / geometry order:
%
%   1 = front-right
%   2 = rear-right
%   3 = rear-left
%   4 = front-left
%
% +x forward
% +y right
% -------------------------------------------------------------------------

dirs = { ...
    [ 1;  1; 0], ...
    [-1;  1; 0], ...
    [-1; -1; 0], ...
    [ 1; -1; 0] ...
};


names_arm = { ...
    "arm_1", ...
    "arm_2", ...
    "arm_3", ...
    "arm_4" ...
};


names_motor = { ...
    "motor_1", ...
    "motor_2", ...
    "motor_3", ...
    "motor_4" ...
};


names_prop = { ...
    "prop_1", ...
    "prop_2", ...
    "prop_3", ...
    "prop_4" ...
};


%% ------------------------------------------------------------------------
% Arms and motors
% -------------------------------------------------------------------------

mA = ...
    P.Arm_Mass;

rA = ...
    P.Arm_Radius;


for i = 1:4

    %% --------------------------------------------------------------------
    % Rotor / arm direction
    % ---------------------------------------------------------------------

    if use_rotor_override

        r_motor_i = ...
            rotor_pos_origin(i,:).';


        du = [ ...
            r_motor_i(1);
            r_motor_i(2);
            0];


        L_i = ...
            norm(du(1:2));


        du = ...
            du / L_i;


        % Simplified arm remains on the calculated arm plane.

        r_arm_com = ...
            0.5*L_i*du + ...
            [0;0;z_arm];

    else

        d = ...
            dirs{i};


        du = ...
            d / norm(d(1:2));


        du = [ ...
            du(1);
            du(2);
            0];


        L_i = ...
            P.Arm_Length;


        r_arm_com = ...
            0.5*L_i*du + ...
            [0;0;z_arm];


        rotor_pos_origin(i,:) = ...
            ( ...
                L_i*du + ...
                [0;0;z_motor] ...
            ).';


        r_motor_i = ...
            rotor_pos_origin(i,:).';

    end


    %% --------------------------------------------------------------------
    % Arm inertia
    %
    % Local x-axis lies along arm.
    % ---------------------------------------------------------------------

    J_long = ...
        0.5 * ...
        mA * ...
        rA^2;


    J_perp = ...
        (1/12) * ...
        mA * ...
        ( ...
            3*rA^2 + ...
            L_i^2 ...
        );


    I_arm_local = ...
        diag([ ...
            J_long, ...
            J_perp, ...
            J_perp]);


    R_arm = ...
        local_make_R(du);


    I_arm_body = ...
        R_arm * ...
        I_arm_local * ...
        R_arm.';


    %% Add arm

    components(end+1).name = ...
        names_arm{i};

    components(end).m = ...
        mA;

    components(end).r = ...
        r_arm_com;

    components(end).Icom = ...
        I_arm_body;


    %% --------------------------------------------------------------------
    % Motor
    % ---------------------------------------------------------------------

    mM = ...
        P.Motor_Mass;

    RM = ...
        P.Motor_Radius;

    HM = ...
        P.Motor_Height;


    I_motor_com = ...
        cyl_inertia_com( ...
            mM, ...
            RM, ...
            HM);


    components(end+1).name = ...
        names_motor{i};

    components(end).m = ...
        mM;

    components(end).r = ...
        r_motor_i;

    components(end).Icom = ...
        I_motor_com;

end


%% ------------------------------------------------------------------------
% Propellers
%
% Currently represented as point masses located at rotor axes.
% -------------------------------------------------------------------------

if include_prop_pointmass

    mP = ...
        P.Propeller_Mass;


    for i = 1:4

        r_prop = ...
            rotor_pos_origin(i,:).';


        if ~use_rotor_override

            r_prop(3) = ...
                z_prop;

        end


        components(end+1).name = ...
            names_prop{i};

        components(end).m = ...
            mP;

        components(end).r = ...
            r_prop;

        components(end).Icom = ...
            zeros(3,3);

    end

end


%% ------------------------------------------------------------------------
% Estimate vehicle COM
% -------------------------------------------------------------------------

Msum = ...
    0;

rc_est = ...
    zeros(3,1);


for i = 1:numel(components)

    Msum = ...
        Msum + ...
        components(i).m;


    rc_est = ...
        rc_est + ...
        components(i).m * ...
        components(i).r;

end


if Msum <= 0

    error( ...
        "Total component mass must be positive.");

end


rc_est = ...
    rc_est / Msum;


%% ------------------------------------------------------------------------
% Select final COM
% -------------------------------------------------------------------------

rc = ...
    rc_est;


if isfield(P.User,'COM') && ...
        ~isempty(P.User.COM)

    rc = ...
        P.User.COM(:);


    if numel(rc) ~= 3

        error( ...
            "P.User.COM must be a 3-element vector.");

    end


    notice( ...
        "Using user-provided COM P.User.COM " + ...
        "(overrides estimated COM).");

else

    notice( ...
        "Estimated COM from component mass distribution.");

end


%% ------------------------------------------------------------------------
% Inertia about final COM
% -------------------------------------------------------------------------

if isfield(P.User,'Inertia_COM') && ...
        ~isempty(P.User.Inertia_COM)

    I_total = ...
        P.User.Inertia_COM;


    if ~isequal(size(I_total), [3 3])

        error( ...
            "P.User.Inertia_COM must be a 3x3 matrix.");

    end


    %% Enforce symmetry if necessary

    if norm( ...
            I_total - I_total.', ...
            'fro') > 1e-10

        notice( ...
            "WARNING: P.User.Inertia_COM is not symmetric. " + ...
            "Symmetrizing.");

        I_total = ...
            0.5 * ...
            (I_total + I_total.');

    end


    %% Positive-definite check

    eigenvalues = ...
        eig(I_total);


    if any(eigenvalues <= 0)

        error( ...
            "P.User.Inertia_COM must be positive definite.");

    end


    notice( ...
        "Using user-provided FULL inertia matrix " + ...
        "P.User.Inertia_COM.");

else

    I_total = ...
        zeros(3,3);


    for i = 1:numel(components)

        r_i = ...
            components(i).r - ...
            rc;


        I_total = ...
            I_total + ...
            parax( ...
                components(i).Icom, ...
                components(i).m, ...
                r_i);

    end


    notice( ...
        "Estimated inertia about COM from geometry + masses " + ...
        "(parallel-axis used).");

end


%% ------------------------------------------------------------------------
% Final symmetry cleanup
% -------------------------------------------------------------------------

I_total = ...
    0.5 * ...
    (I_total + I_total.');


%% ------------------------------------------------------------------------
% Derived results
% -------------------------------------------------------------------------

P.Derived.mass_mode = ...
    string(mass_mode);


%% Mass bookkeeping

P.Derived.m_known = ...
    m_known;

P.Derived.m_arms = ...
    m_arms;

P.Derived.m_motors = ...
    m_motors;

P.Derived.m_props = ...
    m_props;


%% Final mass

P.Derived.Total_Mass = ...
    Msum;

P.Derived.Body_Mass = ...
    P.Body_Mass;


%% Centers

P.Derived.GeomCenter = ...
    r_geom(:).';

P.Derived.COM_Est = ...
    rc_est(:).';

P.Derived.COM = ...
    rc(:).';

P.Derived.COM_Offset = ...
    (rc - r_geom).';


%% Inertia

P.Derived.Inertia_COM = ...
    I_total;

P.Derived.Inertia_COM_Diag = ...
    diag(I_total).';

P.Derived.Inertia_COM_OffDiag_Fro = ...
    norm( ...
        I_total - ...
        diag(diag(I_total)), ...
        'fro');


%% Rotor geometry

P.Derived.Rotor_Positions_Origin = ...
    rotor_pos_origin;

P.Derived.Rotor_Positions_COM = ...
    rotor_pos_origin - ...
    rc(:).';

P.Derived.Arm_Lengths = ...
    vecnorm( ...
        rotor_pos_origin(:,1:2), ...
        2, ...
        2).';


%% Useful z locations

P.Derived.Z.TopBody = ...
    -P.Body_Height/2;

P.Derived.Z.BottomBody = ...
    z_bottom;

P.Derived.Z.ArmAxis = ...
    z_arm;

P.Derived.Z.MotorAxis = ...
    z_motor;

P.Derived.Z.PropellerAxis = ...
    z_prop;


%% ------------------------------------------------------------------------
% Summary
% -------------------------------------------------------------------------

fprintf("\n");
fprintf("--- Geometry/Inertia Summary (X-config) ---\n");


if isfield(P,'Name') && ...
        ~isempty(P.Name)

    fprintf( ...
        "Drone name           : %s\n", ...
        string(P.Name));

end


fprintf( ...
    "Mass mode            : %s\n", ...
    P.Derived.mass_mode);

fprintf( ...
    "Total mass (computed): %.4f kg\n", ...
    P.Derived.Total_Mass);

fprintf( ...
    "Body mass            : %.4f kg\n", ...
    P.Derived.Body_Mass);

fprintf( ...
    "Known parts mass     : %.4f kg\n", ...
    P.Derived.m_known);

fprintf( ...
    "COM used [x y z]     : [%.4f %.4f %.4f] m\n", ...
    P.Derived.COM);

fprintf( ...
    "Geom center [x y z]  : [%.4f %.4f %.4f] m\n", ...
    P.Derived.GeomCenter);

fprintf( ...
    "COM offset [dx dy dz]: [%.4f %.4f %.4f] m\n", ...
    P.Derived.COM_Offset);

fprintf( ...
    "Inertia diag [Jx Jy Jz] = " + ...
    "[%.6f %.6f %.6f] kg*m^2\n", ...
    P.Derived.Inertia_COM_Diag);

fprintf( ...
    "Off-diagonal Fro norm   = %.6e kg*m^2\n", ...
    P.Derived.Inertia_COM_OffDiag_Fro);

fprintf( ...
    "Rotor mean arm length   = %.4f m\n", ...
    mean(P.Derived.Arm_Lengths));

fprintf( ...
    "Rotor z from origin     = %.4f m\n", ...
    mean(P.Derived.Rotor_Positions_Origin(:,3)));

fprintf( ...
    "Full inertia matrix J (about COM) [kg*m^2]:\n");

disp( ...
    P.Derived.Inertia_COM);

fprintf("------------------------------------------\n\n");

end


%% =========================================================================
% Local helper
% =========================================================================

function R = local_make_R(d)
%LOCAL_MAKE_R
% Construct an orthonormal rotation matrix whose local x-axis lies along
% the supplied direction in the body xy-plane.

d = ...
    d(:);

d(3) = ...
    0;


nd = ...
    norm(d);


if nd < 1e-12

    error( ...
        "Direction vector for arm is near zero.");

end


ex = ...
    d / nd;

ez = ...
    [0;0;1];

ey = ...
    cross(ez,ex);


ney = ...
    norm(ey);


if ney < 1e-12

    error( ...
        "Failed to build orthonormal arm basis.");

end


ey = ...
    ey / ney;


R = ...
    [ex ey ez];

end