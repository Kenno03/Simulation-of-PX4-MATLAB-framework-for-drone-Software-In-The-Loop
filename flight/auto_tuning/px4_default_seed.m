function tune=px4_default_seed(current,defaults)
%PX4_DEFAULT_SEED Starting profile of a tuning run: the current setup with every PID gain at Default.
%   tune = px4_default_seed(current,defaults) keeps the initialised aircraft and filter settings of
%   CURRENT (which must be the Default profile) and sets all 23 controller gains to DEFAULTS, so a
%   tuning run never starts from, or towards, the hand-tuned gains.
assert(string(current.ID)=="Default",'Select Default before optimizing.');
tune=current;
for name=px4_controller_gain_names(),tune.(name)=defaults.(name);end
tune.Name=defaults.Name;tune.Source="Default-only cold start; no reference gains";
end
