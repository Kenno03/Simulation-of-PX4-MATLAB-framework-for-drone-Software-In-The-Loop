function ok = check_gain_validity(Tune, SimParams, LQIValidatedFor, options)
%CHECK_GAIN_VALIDITY Warn when the selected controller gains were not validated on the current setup.
%   ok = check_gain_validity(PX4Tune, SimParams, LQIValidatedFor) compares the setup the selected gains
%   were tuned or designed for with the current vehicle (drone, motors, propellers and payload):
%     - total mass,
%     - inertia about the body axes,
%     - maximum total thrust (motors and propellers set the control authority).
%   If any of them differs by more than 10 %, a warning says that the gains have not been validated on
%   this setup. ok is false when the warning was issued.
%
%   PX4Tune         px4_tuning_parameters profile. Its ValidatedFor field holds the validated setup;
%                   empty for generic gains (the PX4 defaults), which never warn.
%   SimParams       build_simulation_parameters of the current setup.
%   LQIValidatedFor second output of custom_controller_parameters; used when PX4Tune.CustomController
%                   is 1 (the custom LQI controller flies).
%   Tolerance       relative difference that triggers the warning (default 0.10).
%
%   Called by Main.mlx after the controller profile is loaded and by run_mission.
arguments
    Tune (1,1) struct
    SimParams (1,1) struct
    LQIValidatedFor = []
    options.Tolerance (1,1) double {mustBePositive} = 0.10
end
if isfield(Tune,'CustomController') && Tune.CustomController
    V = LQIValidatedFor; what = "custom LQI controller gains";
else
    V = []; if isfield(Tune,'ValidatedFor'), V = Tune.ValidatedFor; end
    what = "PX4 gains """ + string(Tune.ID) + """";
end
ok = true;
if isempty(V), return; end          % generic gains (PX4 defaults): valid for any vehicle

J = SimParams.Inertia; if ~isvector(J), J = diag(J); end
current = [SimParams.TotalMass, J(:)', SimParams.CommandMaxTotalThrust];
if ~isfinite(V.Mass)
    msg = sprintf("The selected %s have not been validated on this setup: they were tuned for a different vehicle.", what);
else
    validated = [V.Mass, V.Inertia(:)', V.MaxThrust];
    d = current./validated - 1;
    if all(abs(d) <= options.Tolerance), return; end
    msg = sprintf(['The selected %s have not been validated on this setup (more than %.0f %% difference):\n' ...
        '  mass        %.3f kg (validated %.3f kg, %+.0f %%)\n' ...
        '  inertia     [%.4g %.4g %.4g] kg*m^2 (validated [%.4g %.4g %.4g], %+.0f / %+.0f / %+.0f %%)\n' ...
        '  max thrust  %.1f N (validated %.1f N, %+.0f %%)'], ...
        what, 100*options.Tolerance, current(1), validated(1), 100*d(1), current(2:4), validated(2:4), 100*d(2:4), ...
        current(5), validated(5), 100*d(5));
end
msg = strjoin(string(msg), "") + newline + "Expect different flight behaviour. Retune " + ...
    "(run_px4_guide_tuning) or redesign (design_custom_controller) for this setup.";
warning('SITL:gainsNotValidated', '%s', char(msg));
ok = false;
end
