%START_PX4_TUNING Start a tuning session from the numbered menu in the Command Window.
%   Type start_px4_tuning (or run Main.mlx's tuning section) and choose a group, a single loop, the
%   guide method or the RMSE method from the menu (px4_tuning_menu). The session runs with a 48 h wall
%   budget and its result is left in the base workspace as TuningSession.
TuningSession=run_px4_tuning(MaxHours=48);
