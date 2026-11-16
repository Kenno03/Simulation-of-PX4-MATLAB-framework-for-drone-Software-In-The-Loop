function stop_px4_run(watcher)
%STOP_PX4_RUN Stop plant, PX4 SITL and PX4 CMD console.
%
% Preserves watcher trace and error files for diagnosis.
%
% Throws an error if Simulink or PX4 cleanup is incomplete,
% so runCandidate can stop instead of starting another
% simulation on top of an existing PX4 instance.

    if nargin < 1
        watcher = [];
    end

    fprintf('\n========================================\n');
    fprintf(' Stopping PX4 Simulation\n');
    fprintf('========================================\n');

    cleanupErrors = strings(0,1);

    %% 1. Stop Simulink models

    models = [
        "Drone_simulation_PX4"
        "PX4_Receive"
    ];

    for i = 1:numel(models)

        mdl = models(i);

        try

            if ~bdIsLoaded(mdl)
                continue;
            end

            state = string(get_param( ...
                mdl,'SimulationStatus'));

            if state ~= "stopped"

                fprintf('Stopping %s...\n',mdl);

                set_param(mdl, ...
                    'SimulationCommand','stop');

            end

        catch ME

            warning('Could not stop %s: %s', ...
                mdl,ME.message);

            cleanupErrors(end+1) = ...
                "Model stop: " + mdl + ": " + ME.message;

        end

    end

    %% 2. Wait for both models to stop

    allStopped = false;

    for k = 1:60

        allStopped = true;

        for i = 1:numel(models)

            mdl = models(i);

            if bdIsLoaded(mdl)

                try

                    state = string(get_param( ...
                        mdl,'SimulationStatus'));

                    if state ~= "stopped"
                        allStopped = false;
                    end

                catch
                    allStopped = false;
                end

            end

        end

        if allStopped
            break;
        end

        pause(0.25);

    end

    if ~allStopped

        cleanupErrors(end+1) = ...
            "Simulink models did not stop.";

        warning('Simulink models did not stop within 15 seconds.');

    end

    %% 3. Stop PX4 SITL in WSL

    fprintf('Terminating PX4 SITL...\n');

    try

        [~,msg] = system( ...
            'wsl.exe -e pkill -TERM -x px4');

        if strlength(strtrim(string(msg))) > 0
            fprintf('%s\n',strtrim(msg));
        end

        pause(2);

        %% Check whether PX4 is still running

        [running,~] = system( ...
            'wsl.exe -e pgrep -x px4');

        if running == 0

            fprintf('PX4 still running; forcing termination.\n');

            system('wsl.exe -e pkill -KILL -x px4');

            pause(1);

            [running,~] = system( ...
                'wsl.exe -e pgrep -x px4');

        end

        if running == 0

            cleanupErrors(end+1) = ...
                "PX4 SITL remains running.";

            warning('PX4 SITL could not be terminated.');

        else

            fprintf('PX4 SITL process terminated.\n');

        end

    catch ME

        cleanupErrors(end+1) = ...
            "WSL cleanup: " + string(ME.message);

        warning('WSL cleanup failed: %s',ME.message);

    end

    %% 4. Close identifiable PX4 CMD windows

    fprintf('Closing PX4 CMD window...\n');

    psFile = [tempname(tempdir) '.ps1'];

    try

        psLines = {

'$ErrorActionPreference = "SilentlyContinue"'
'$closed = 0'
'$cmds = Get-CimInstance Win32_Process -Filter ''Name="cmd.exe"'''
'foreach ($c in $cmds) {'
'    $p = Get-Process -Id $c.ProcessId -ErrorAction SilentlyContinue'
'    $title = ""'
'    if ($p) { $title = [string]$p.MainWindowTitle }'
'    $line = [string]$c.CommandLine'
'    $titleMatch = $title -match "(?i)PX4"'
'    $commandMatch = $line -match "(?i)PX4-Autopilot|px4_sitl"'
'    if ($titleMatch -or $commandMatch) {'
'        Stop-Process -Id $c.ProcessId -Force -ErrorAction SilentlyContinue'
'        $closed++'
'    }'
'}'
'Write-Output "PX4 CMD processes closed: $closed"'
        };

        fid = fopen(psFile,'wt');

        assert(fid ~= -1, ...
            'Unable to create CMD cleanup script.');

        for k = 1:numel(psLines)
            fprintf(fid,'%s\n',psLines{k});
        end

        fclose(fid);

        cmd = sprintf( ...
            ['powershell.exe -NoProfile ' ...
             '-NonInteractive ' ...
             '-ExecutionPolicy Bypass ' ...
             '-File "%s"'], ...
            psFile);

        [closeStatus,closeMessage] = system(cmd);

        if strlength(strtrim(string(closeMessage))) > 0
            fprintf('%s\n',strtrim(closeMessage));
        end

        if closeStatus ~= 0

            cleanupErrors(end+1) = ...
                "PX4 CMD cleanup failed.";

            warning('Could not finish CMD cleanup.');

        end

    catch ME

        cleanupErrors(end+1) = ...
            "CMD cleanup: " + string(ME.message);

        warning('CMD cleanup failed: %s',ME.message);

    end

    if isfile(psFile)
        delete(psFile);
    end

    %% 5. Read watcher result and diagnostics

    if ~isempty(watcher) && isstruct(watcher)

        acknowledged = false;
        watcherError = "";

        if isfield(watcher,'StatusFile') && ...
           isfile(watcher.StatusFile)

            acknowledged = true;

            message = strtrim( ...
                fileread(watcher.StatusFile));

            fprintf('PX4 popup closed: %s\n',message);

        end

        if isfield(watcher,'ErrorFile') && ...
           isfile(watcher.ErrorFile)

            watcherError = string(strtrim( ...
                fileread(watcher.ErrorFile)));

            warning('Popup watcher: %s',watcherError);

            if contains(watcherError, ...
                    "could not be acknowledged") || ...
               contains(watcherError, ...
                    "after detecting the popup")

                cleanupErrors(end+1) = ...
                    "Popup watcher failed: " + watcherError;

            end

        end

        %% Show recent watcher trace

        if isfield(watcher,'TraceFile') && ...
           isfile(watcher.TraceFile)

            fprintf('\nPX4 POPUP WATCHER TRACE\n');

            traceLines = splitlines( ...
                string(fileread(watcher.TraceFile)));

            traceLines = traceLines( ...
                strlength(strtrim(traceLines)) > 0);

            n = numel(traceLines);

            first = max(1,n-12);

            for k = first:n
                fprintf('  %s\n',traceLines(k));
            end

            fprintf('Trace saved at:\n%s\n', ...
                watcher.TraceFile);

        end

        %% Request independent watcher shutdown

        if isfield(watcher,'StopFile')

            fid = fopen(watcher.StopFile,'wt');

            if fid ~= -1
                fclose(fid);
            end

        end

        %% Do not confuse "not acknowledged" with
        % a confirmed watcher failure.

        if ~acknowledged && strlength(watcherError)==0

            fprintf(['Popup acknowledgement not recorded. ', ...
                'Check the watcher trace above.\n']);

        end

    elseif ~isempty(watcher) && ...
           isa(watcher,'timer') && isvalid(watcher)

        if strcmp(watcher.Running,'on')
            stop(watcher);
        end

        delete(watcher);

    end

    fprintf('\nPX4 cleanup finished.\n');

    %% 6. Report incomplete cleanup as a real error

    if ~isempty(cleanupErrors)

        error('stop_px4_run:Incomplete', ...
            'PX4 cleanup incomplete: %s', ...
            strjoin(cleanupErrors,' | '));

    end

end
