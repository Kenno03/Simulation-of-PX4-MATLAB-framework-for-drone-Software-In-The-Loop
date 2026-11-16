function search=px4_joint_initialize(S,defaults)
%PX4_JOINT_INITIALIZE Start state of the joint finite-difference gain search for one loop (run_px4_tuning).
%   search = px4_joint_initialize(Stage,defaults) scales the loop's gains by their Default values into
%   dimensionless coordinates, sets the bounds and step radius, and queues the first probes.
%   See px4_joint_next (next proposal) and px4_joint_observe (record the flown score).
% Dimensionless controller coordinates. K is fixed; PID gains are effective gains.
names=string(S.Parameters);initial=arrayfun(@(n)double(defaults.(n)),names);
scale=initial;scale(scale==0)=.1;
for k=1:numel(names)
    if initial(k)==0&&contains(names(k),"RATE_D"),scale(k)=1e-4;end
end
lower=zeros(size(initial));upper=zeros(size(initial));
for k=1:numel(names)
    [lo,hi]=px4_controller_gain_limits(names(k));
    if initial(k)>0,cap=5*initial(k);else,cap=.5;end
    if contains(names(k),"RATE_D")&&initial(k)==0,cap=1e-3;end
    lower(k)=lo/scale(k);upper(k)=min(hi,cap)/scale(k);
    if ~endsWith(names(k),["_I","_D","_I_ACC","_D_ACC"])
        lower(k)=max(lower(k),.1*initial(k)/scale(k));
    end
end
search=struct('Version',"joint_fd_v1",'Names',names,'Scale',scale, ...
    'Lower',lower,'Upper',upper,'Initial',initial./scale,'Radius',.2, ...
    'Phase',"seed",'SeedIndex',1,'ProbeIndex',1,'CompletedCycles',0, ...
    'CycleComplete',false,'CycleImproved',false,'Center',[], ...
    'CenterScore',NaN,'ProbeX',[],'ProbeScores',[],'Pending',struct());
end
