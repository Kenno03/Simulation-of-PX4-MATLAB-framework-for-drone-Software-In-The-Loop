function watcher = px4_ack_popup(timeoutSeconds)
%PX4_ACK_POPUP Independent Windows popup watcher for PX4 SITL.
%
% Targets only windows whose title contains:
%   "Start Simulink plant"
%
% Methods:
%   1. Windows button message (BM_CLICK)
%   2. Windows dialog OK command (WM_COMMAND/IDOK)
%   3. UI Automation InvokePattern
%   4. Targeted keyboard Enter
%
% Remains active after acknowledgment, since Monitor & Tune can show the
% dialog again after rebuilding. StopFile or timeout ends the watcher.
%
% Outputs:
%   StatusFile - confirmed popup dismissed
%   ErrorFile  - watcher failed
%   ReadyFile  - watcher initialized
%   TraceFile  - diagnostics
%   StopFile   - shutdown request
%
% Requires an interactive Windows user session.
% Does not guarantee operation during Windows sleep.

    if nargin < 1
        timeoutSeconds = 900;
    end

    assert(ispc, ...
        'px4_ack_popup requires Windows.');

    validateattributes(timeoutSeconds, ...
        {'numeric'}, ...
        {'scalar','positive','finite'});

    %% Unique temporary files

    base = char(tempname(tempdir));

    scriptFile = [base '.ps1'];
    statusFile = [base '.ack'];
    stopFile   = [base '.stop'];
    errorFile  = [base '.error'];
    readyFile  = [base '.ready'];
    traceFile  = [base '.log'];

    %% PowerShell watcher source

    ps = {

'param('
'    [string]$StatusFile,'
'    [string]$StopFile,'
'    [string]$ErrorFile,'
'    [string]$ReadyFile,'
'    [string]$TraceFile,'
'    [int]$TimeoutSeconds = 900'
')'
''
'$ErrorActionPreference = "Stop"'
'$title = "Start Simulink plant"'
'$maxPopupSeconds = 30'
''
'function Trace([string]$message) {'
'    try {'
'        Add-Content -LiteralPath $TraceFile -Value ((Get-Date -Format "HH:mm:ss.fff") + " " + $message)'
'    } catch {}'
'}'
''
'# Native Win32 access, independent of window focus.'
'$nativeCode = @"'
'using System;'
'using System.Text;'
'using System.Runtime.InteropServices;'
''
'public static class Px4PopupNative {'
'    public delegate bool WindowCallback(IntPtr h, IntPtr p);'
''
'    [DllImport("user32.dll")]'
'    public static extern bool EnumWindows(WindowCallback cb, IntPtr p);'
''
'    [DllImport("user32.dll", CharSet=CharSet.Unicode)]'
'    public static extern int GetWindowText(IntPtr h, StringBuilder b, int n);'
''
'    [DllImport("user32.dll")]'
'    public static extern bool IsWindowVisible(IntPtr h);'
''
'    [DllImport("user32.dll", CharSet=CharSet.Unicode)]'
'    public static extern IntPtr FindWindowEx(IntPtr parent, IntPtr after, string cls, string name);'
''
'    [DllImport("user32.dll")]'
'    public static extern bool PostMessage(IntPtr h, uint msg, IntPtr w, IntPtr l);'
''
'    [DllImport("user32.dll")]'
'    public static extern bool SetForegroundWindow(IntPtr h);'
''
'    [DllImport("user32.dll")]'
'    public static extern IntPtr GetForegroundWindow();'
''
'    public static IntPtr FindTitle(string title) {'
'        IntPtr found = IntPtr.Zero;'
'        EnumWindows((h,p) => {'
'            if (!IsWindowVisible(h)) return true;'
'            StringBuilder text = new StringBuilder(512);'
'            GetWindowText(h,text,text.Capacity);'
'            if (text.ToString().IndexOf(title, StringComparison.OrdinalIgnoreCase) >= 0) {'
'                found = h;'
'                return false;'
'            }'
'            return true;'
'        },IntPtr.Zero);'
'        return found;'
'    }'
'}'
'"@'
''
'try {'
'    Add-Type -TypeDefinition $nativeCode -ErrorAction Stop'
''
'    $uiaAvailable = $true'
'    try {'
'        Add-Type -AssemblyName UIAutomationClient'
'        Add-Type -AssemblyName UIAutomationTypes'
'    } catch {'
'        $uiaAvailable = $false'
'    }'
''
'    $shell = $null'
'    try {'
'        $shell = New-Object -ComObject WScript.Shell'
'    } catch {}'
''
'    Trace "Watcher ready. UIA=$uiaAvailable"'
'    Set-Content -LiteralPath $ReadyFile -Value "READY"'
'}'
'catch {'
'    Set-Content -LiteralPath $ErrorFile -Value $_.Exception.Message'
'    exit 2'
'}'
''
'function Find-Popup {'
'    return [Px4PopupNative]::FindTitle($title)'
'}'
''
'function Try-UiaClick {'
'    if (-not $uiaAvailable) { return $false }'
'    try {'
'        $root = [System.Windows.Automation.AutomationElement]::RootElement'
'        $windows = $root.FindAll('
'            [System.Windows.Automation.TreeScope]::Children,'
'            [System.Windows.Automation.Condition]::TrueCondition)'
''
'        foreach ($window in $windows) {'
'            if ($window.Current.Name -notlike "*Start Simulink plant*") { continue }'
''
'            $buttonCondition = [System.Windows.Automation.PropertyCondition]::new('
'                [System.Windows.Automation.AutomationElement]::ControlTypeProperty,'
'                [System.Windows.Automation.ControlType]::Button)'
''
'            $buttons = $window.FindAll('
'                [System.Windows.Automation.TreeScope]::Descendants,'
'                $buttonCondition)'
''
'            foreach ($button in $buttons) {'
'                $name = [string]$button.Current.Name'
'                if ($name -notmatch "^(OK|&OK|Continue)$") { continue }'
''
'                $pattern = $button.GetCurrentPattern('
'                    [System.Windows.Automation.InvokePattern]::Pattern)'
'                $pattern.Invoke()'
'                Trace "UI Automation invoked button: $name"'
'                return $true'
'            }'
'        }'
'    }'
'    catch {'
'        Trace ("UIA error: " + $_.Exception.Message)'
'    }'
'    return $false'
'}'
''
'function Confirm-Closed([string]$method) {'
'    if ((Find-Popup) -eq [IntPtr]::Zero) {'
'        Set-Content -LiteralPath $StatusFile -Value $method'
'        Trace ("SUCCESS: " + $method)'
'        return $true'
'    }'
'    return $false'
'}'
''
'$clock = [System.Diagnostics.Stopwatch]::StartNew()'
'$firstSeen = -1.0'
'$lastReport = -5.0'
''
'while ($clock.Elapsed.TotalSeconds -lt $TimeoutSeconds) {'
''
'    if (Test-Path -LiteralPath $StopFile) {'
'        Trace "Shutdown requested by MATLAB."'
'        exit 0'
'    }'
''
'    $h = Find-Popup'
''
'    if ($h -ne [IntPtr]::Zero) {'
''
'        if ($firstSeen -lt 0) {'
'            $firstSeen = $clock.Elapsed.TotalSeconds'
'            Trace "Target popup detected."'
'        }'
''
'        # Method 1: click the actual Windows OK button.'
'        $button = [Px4PopupNative]::FindWindowEx('
'            $h,[IntPtr]::Zero,"Button","OK")'
''
'        if ($button -ne [IntPtr]::Zero) {'
'            [void][Px4PopupNative]::PostMessage('
'                $button,0x00F5,[IntPtr]::Zero,[IntPtr]::Zero)'
'            Start-Sleep -Milliseconds 250'
'            if (Confirm-Closed "Win32 BM_CLICK") { $firstSeen = -1; continue }'
'        }'
''
'        # Method 2: send IDOK to the selected dialog.'
'        [void][Px4PopupNative]::PostMessage('
'            $h,0x0111,[IntPtr]1,[IntPtr]::Zero)'
''
'        Start-Sleep -Milliseconds 250'
'        if (Confirm-Closed "Win32 WM_COMMAND") { $firstSeen = -1; continue }'
''
'        # Method 3: UI Automation.'
'        [void](Try-UiaClick)'
''
'        Start-Sleep -Milliseconds 250'
'        if (Confirm-Closed "UI Automation") { $firstSeen = -1; continue }'
''
'        # Method 4: Enter only when this exact'
'        # popup is the foreground window.'
'        try {'
'            if ($null -ne $shell) {'
'                [void][Px4PopupNative]::SetForegroundWindow($h)'
'                Start-Sleep -Milliseconds 100'
'                if ([Px4PopupNative]::GetForegroundWindow() -eq $h) {'
'                    $shell.SendKeys("{ENTER}")'
'                    Trace "Sent Enter to target popup."'
'                }'
'            }'
'        } catch {'
'            Trace ("Keyboard error: " + $_.Exception.Message)'
'        }'
''
'        Start-Sleep -Milliseconds 250'
'        if (Confirm-Closed "Targeted Enter") { $firstSeen = -1; continue }'
''
'        # Prevent a permanently blocked startup.'
'        if (($clock.Elapsed.TotalSeconds - $firstSeen) -ge $maxPopupSeconds) {'
'            Trace "Popup acknowledgement failed; sending WM_CLOSE."'
'            [void][Px4PopupNative]::PostMessage('
'                $h,0x0010,[IntPtr]::Zero,[IntPtr]::Zero)'
'            Set-Content -LiteralPath $ErrorFile -Value "Popup could not be acknowledged within 30 seconds."'
'            exit 1'
'        }'
'    }'
''
'    if (($clock.Elapsed.TotalSeconds - $lastReport) -ge 5) {'
'        Trace ("Watching. Popup visible=" + ($h -ne [IntPtr]::Zero))'
'        $lastReport = $clock.Elapsed.TotalSeconds'
'    }'
''
'    Start-Sleep -Milliseconds 300'
'}'
''
'if (Test-Path -LiteralPath $StatusFile) { exit 0 }'
'if ($firstSeen -lt 0) {'
'    $message = "Watcher timed out; target popup was never detected."'
'} else {'
'    $message = "Watcher timed out after detecting the popup."'
'}'
'Trace $message'
'Set-Content -LiteralPath $ErrorFile -Value $message'
'exit 1'
    };

    %% Write script

    fid = fopen(scriptFile,'wt');

    assert(fid ~= -1, ...
        'Unable to create popup watcher script.');

    for k = 1:numel(ps)
        fprintf(fid,'%s\n',ps{k});
    end

    fclose(fid);

    %% Start watcher as independent Windows process

    cmd = sprintf( ...
        ['cmd.exe /d /c start "" powershell.exe ' ...
         '-NoProfile -STA ' ...
         '-ExecutionPolicy Bypass ' ...
         '-WindowStyle Hidden ' ...
         '-File "%s" ' ...
         '-StatusFile "%s" ' ...
         '-StopFile "%s" ' ...
         '-ErrorFile "%s" ' ...
         '-ReadyFile "%s" ' ...
         '-TraceFile "%s" ' ...
         '-TimeoutSeconds %d'], ...
        scriptFile,statusFile,stopFile, ...
        errorFile,readyFile,traceFile, ...
        ceil(timeoutSeconds));

    [rc,output] = system(cmd);

    assert(rc == 0, ...
        'Popup watcher launch failed: %s',output);

    %% Wait for watcher readiness

    startupClock = tic;

    while toc(startupClock) < 15

        if isfile(readyFile)
            break;
        end

        if isfile(errorFile)
            error('Popup watcher initialization failed: %s', ...
                strtrim(fileread(errorFile)));
        end

        pause(0.1);
    end

    assert(isfile(readyFile), ...
        'Popup watcher did not initialize in 15 seconds.');

    %% Return watcher information

    watcher = struct( ...
        'StatusFile',statusFile, ...
        'StopFile',stopFile, ...
        'ErrorFile',errorFile, ...
        'ReadyFile',readyFile, ...
        'TraceFile',traceFile, ...
        'ScriptFile',scriptFile);

    fprintf('PX4 popup watcher initialized.\n');
    fprintf('Watcher trace: %s\n',traceFile);

end
