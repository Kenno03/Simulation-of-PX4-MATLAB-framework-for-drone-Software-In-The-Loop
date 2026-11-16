function px4_session_lock(action)
%PX4_SESSION_LOCK Allow only one tuning or flight run per MATLAB session.
%   px4_session_lock("acquire") errors if another run already holds the lock;
%   px4_session_lock("release") frees it (the runners release it with onCleanup).
persistent active
if isempty(active),active=false;end
if action=="acquire"
    assert(~active,'A tuning function already owns this MATLAB session. Wait for it to finish.');
    active=true;
else
    active=false;
end
end
