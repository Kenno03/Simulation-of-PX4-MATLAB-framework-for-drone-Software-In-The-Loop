function waited=px4_tuning_wait_for_resume(folder)
%PX4_TUNING_WAIT_FOR_RESUME Pause a tuning session between trials while <folder>/pause.request exists.
%   waited = px4_tuning_wait_for_resume(folder) returns the time waited [s]; the runner extends its
%   wall budget by that time.
% A pause.request file pauses BETWEEN trials; removing it resumes the session.
% This lets an unattended run wait for quota refresh without losing its state.
waited=0;pauseFile=fullfile(folder,'pause.request');
if ~isfile(pauseFile),return;end
fprintf('Checkpoint saved. Paused between trials; remove %s to resume.\n',pauseFile);
clock=tic;
while isfile(pauseFile),pause(.5);end
waited=toc(clock);fprintf('Resuming saved tuning session.\n');
end
