function command=px4_tuning_step_command(testTime,mode,amplitude)
%PX4_TUNING_STEP_COMMAND Step-test command at time testTime: +/- amplitude inside a pulse, else 0.
%   command = px4_tuning_step_command(testTime,mode,amplitude) uses the pulse schedule of
%   px4_tuning_step_windows(mode). Runs in PX4_Receive (code generation).
%#codegen
[windows,signs]=px4_tuning_step_windows(mode);
command=single(0);
for k=1:10
    if testTime>=windows(k,1)&&testTime<windows(k,2)
        command=single(signs(k))*single(amplitude);
    end
end
end
