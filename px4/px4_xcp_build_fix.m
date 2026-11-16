function px4_xcp_build_fix(model)
%PX4_XCP_BUILD_FIX Restore the stock MATLAB/PX4 XCP build settings.
% PX4's generated main uses the modern XCP API. Keep the standard PX4 target
% hook, but select the matching modern interface in TLC. Do not inject or
% replace XCP sources.
options=get_param(model,'TLCOptions');
options=regexprep(options,'-aExtModeXCPClassicInterface=\S+','');
set_param(model,'TLCOptions',strtrim([options ' -aExtModeXCPClassicInterface=0']));

set_param(model,'PostCodeGenCommand','codertarget.postCodeGenHookCommand(h)');
codertarget.data.setParameterValue(model,'Runtime.BuildAction','Build, load and run');

source=fullfile(matlabroot,'toolbox','coder','xcp','src','target','ext_mode','src','xcp_ext_mode.c');
custom=get_param(model,'CustomSource');
custom=strtrim(strrep(custom,['"' source '"'],''));
set_param(model,'CustomSource',custom);
end
