function names=configure_px4_receive_for_batch(model,options)
%CONFIGURE_PX4_RECEIVE_FOR_BATCH Prepare PX4_Receive for an automated (unattended) run.
%   names = configure_px4_receive_for_batch(model)
%   configure_px4_receive_for_batch(model,ShowScopes=true)   % keep the live scopes (interactive use)
%
%   1. Logs the status output of every PX4 parameter writer (PX4_WRITE_STATUS_<gain>), so each run
%      records that the gains were written. This observes the write status; it is not a readback.
%   2. Comments out the Scope blocks of PX4_Receive unless ShowScopes is true. In external mode every
%      scope streams its data from PX4 on each step; with the 9 scopes off a figure-8 flight runs in
%      141 s instead of 297 s, with identical results. Scopes only display data, so nothing that is
%      logged or scored is lost.
%   Both changes are made in memory only; the runners never save PX4_Receive, and px4_session_cleanup
%   switches the scopes back on. Returns the gain names whose writer status is logged.
arguments
    model
    options.ShowScopes (1,1) logical = false
end
names=px4_controller_gain_names();
for name=names
    lines=find_system(char(model),'FindAll','on','LookUnderMasks','all', ...
        'FollowLinks','on','Type','line','Name',char(name));
    ports=[];
    for k=1:numel(lines)
        port=get_param(lines(k),'SrcPortHandle');
        if port~=-1,ports(end+1)=port;end %#ok<AGROW>
    end
    ports=unique(ports);
    assert(numel(ports)==1,'Expected one status output named %s.',name);
    set_param(ports,'DataLogging','on','DataLoggingNameMode','Custom', ...
        'DataLoggingName',char("PX4_WRITE_STATUS_"+name));
    Simulink.sdi.markSignalForStreaming( ...
        get_param(ports,'Parent'),get_param(ports,'PortNumber'),'on');
end
scopes=find_system(char(model),'LookUnderMasks','all','FollowLinks','on','BlockType','Scope');
state='on';if options.ShowScopes,state='off';end
for k=1:numel(scopes),set_param(scopes{k},'Commented',state);end
end
