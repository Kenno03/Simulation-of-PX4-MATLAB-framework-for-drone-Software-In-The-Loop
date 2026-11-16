function CommParams = communication_parameters()
%COMMUNICATION_PARAMETERS
% Defines communication and sensor update rates used by the PX4 / MAVLink
% simulation.
%
% Frequencies are defined in Hz.
% Sample times are calculated automatically as:
%
%   Ts = 1 / Hz
%
% This keeps one source of truth for all communication timing.

CommParams = struct();

%% ------------------------------------------------------------------------
% INS
% -------------------------------------------------------------------------

CommParams.INS.Hz = 250;                     % [Hz]
CommParams.INS.Ts = 1 / CommParams.INS.Hz;  % [s]


%% ------------------------------------------------------------------------
% MAVLink HIL_SENSOR
% -------------------------------------------------------------------------

CommParams.HIL_SENSOR.Hz = 250;                        % [Hz]
CommParams.HIL_SENSOR.Ts = 1 / CommParams.HIL_SENSOR.Hz; % [s]


%% ------------------------------------------------------------------------
% MAVLink HIL_GPS
% -------------------------------------------------------------------------

CommParams.HIL_GPS.Hz = 250;                     % [Hz]
CommParams.HIL_GPS.Ts = 1 / CommParams.HIL_GPS.Hz; % [s]


%% ------------------------------------------------------------------------
% MAVLink HIL_STATE_QUATERNION
% -------------------------------------------------------------------------

CommParams.HIL_STATE_QUATERNION.Hz = 250; ...
                                                    % [Hz]

CommParams.HIL_STATE_QUATERNION.Ts = ...
    1 / CommParams.HIL_STATE_QUATERNION.Hz;        % [s]


%% ------------------------------------------------------------------------
% MAVLink HEARTBEAT
% -------------------------------------------------------------------------

CommParams.HIL_HEARTBEAT.Hz = 1; ...
                                                    % [Hz]

CommParams.HIL_HEARTBEAT.Ts = ...
    1 / CommParams.HIL_HEARTBEAT.Hz;               % [s]


%% ------------------------------------------------------------------------
% General HIL sample rate
% -------------------------------------------------------------------------
%
% Used where the model requires one common HIL sample time.

CommParams.HIL.Hz = 250;                     % [Hz]
CommParams.HIL.Ts = 1 / CommParams.HIL.Hz;  % [s]


%% ------------------------------------------------------------------------
% Validation
% -------------------------------------------------------------------------

fields = { ...
    CommParams.INS, ...
    CommParams.HIL_SENSOR, ...
    CommParams.HIL_GPS, ...
    CommParams.HIL_STATE_QUATERNION, ...
    CommParams.HIL_HEARTBEAT, ...
    CommParams.HIL ...
};

for k = 1:numel(fields)

    if fields{k}.Hz <= 0 || ~isfinite(fields{k}.Hz)
        error("Communication frequencies must be finite and positive.");
    end

end


%% ------------------------------------------------------------------------
% Summary
% -------------------------------------------------------------------------

fprintf("\n");
fprintf("========================================\n");
fprintf(" Communication Parameters\n");
fprintf("========================================\n");

fprintf(" INS                  : %6.1f Hz  (Ts = %.4f s)\n", ...
    CommParams.INS.Hz, ...
    CommParams.INS.Ts);

fprintf(" HIL_SENSOR           : %6.1f Hz  (Ts = %.4f s)\n", ...
    CommParams.HIL_SENSOR.Hz, ...
    CommParams.HIL_SENSOR.Ts);

fprintf(" HIL_GPS              : %6.1f Hz  (Ts = %.4f s)\n", ...
    CommParams.HIL_GPS.Hz, ...
    CommParams.HIL_GPS.Ts);

fprintf(" HIL_STATE_QUATERNION : %6.1f Hz  (Ts = %.4f s)\n", ...
    CommParams.HIL_STATE_QUATERNION.Hz, ...
    CommParams.HIL_STATE_QUATERNION.Ts);

fprintf(" HIL_HEARTBEAT        : %6.1f Hz  (Ts = %.4f s)\n", ...
    CommParams.HIL_HEARTBEAT.Hz, ...
    CommParams.HIL_HEARTBEAT.Ts);

fprintf(" General HIL          : %6.1f Hz  (Ts = %.4f s)\n", ...
    CommParams.HIL.Hz, ...
    CommParams.HIL.Ts);

fprintf("========================================\n\n");

end