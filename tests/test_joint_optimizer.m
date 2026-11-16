%TEST_JOINT_OPTIMIZER Offline regression checks of the single-loop joint gain search (no simulation).
%   Run test_joint_optimizer. It checks the Default-only seed, the gain bounds, the fixed rate K, the
%   probe/replay logic, a coupled synthetic valley, infeasible probes and the joint response score.
%   No model execution or hand reference is required.
p=fileparts(fileparts(mfilename('fullpath')));
addpath(p);project_paths(p);
D=px4_tuning_parameters("Default"); altered=D;
for n=px4_controller_gain_names(),altered.(n)=9999;end
seed=px4_default_seed(altered,D);
for n=px4_controller_gain_names(),assert(isequal(seed.(n),D.(n)));end
% All non-gain initialization except human-readable provenance is untouched.
for n=string(fieldnames(D))'
 if ~ismember(n,[px4_controller_gain_names(),"Name","Source"]),assert(isequal(seed.(n),D.(n)));end
end
opt=struct('Mode',"tune",'MaxTrials',30,'ShowFigures',false,'MinimumImprovement',.01);
[cfg,stages]=px4_session_configuration("all",opt);
assert(numel(stages)==10&&numel([stages.Parameters])==20&&numel([stages.FixedParameters])==3);
assert(cfg.HomeNED(3)==-10&&cfg.MinimumSteps==5&&cfg.ExpectedSteps==10);
for S=stages
 q=px4_joint_initialize(S,D);assert(all(q.Lower<=q.Initial&q.Initial<=q.Upper));
 [q,g,~,~]=px4_joint_next(q,D,1);assert(all(isfinite(q.Pending.X)));
 [replay,same,~,~]=px4_joint_next(q,D,1);assert(isequal(g,same)&&isequaln(q,replay));
 q=px4_joint_observe(q,1,true,false);
 [q,g,~,~]=px4_joint_next(q,D,1);
 for n=string(S.Parameters)
  if endsWith(n,["_I","_I_ACC"]),[lo,~]=px4_controller_gain_limits(n);assert(abs(g.(n)-lo)<1e-12);end
 end
 for n=string(S.FixedParameters),assert(g.(n)==D.(n));end
end
% Coupled synthetic valley: each coordinate move raises cost, a diagonal move lowers it.
S=stages(1);q=px4_joint_initialize(S,D);q.Phase="cycle";q.SeedIndex=3;
best=D;score=10;seenJoint=false;
for k=1:7
 [q,g,label,changed]=px4_joint_next(q,best,score);
 x=arrayfun(@(n)g.(n),q.Names)./q.Scale;
 u=x(1)-1;v=x(2)-1;z=x(3)-1;
 J=10+100*(u-v)^2+100*z^2-u-v;
 accept=J<score*.99;
 if accept,best=g;score=J;end
 q=px4_joint_observe(q,J,true,accept);
 if label=="Joint gradient update",seenJoint=true;assert(numel(changed)>=2&&score<10);end
 if k<7,assert(~q.CycleComplete);end
end
assert(seenJoint&&q.CycleComplete&&q.CompletedCycles==1);
% Infeasible probes are recorded; an execution fault instead leaves Pending for replay.
q=px4_joint_initialize(S,D);q.Phase="cycle";
for k=1:7
 [q,~,~,~]=px4_joint_next(q,D,1);q=px4_joint_observe(q,Inf,false,false);
end
assert(q.CompletedCycles==1&&q.Radius==.1);
% Objective response checks independent of any gain/profile values.
t=(0:.01:12)';r=zeros(size(t));r(t>=1&t<3)=1;r(t>=5&t<7)=-1;
y=filter(1-exp(-.01/.08),[1 -exp(-.01/.08)],r);windows=[1 3;5 7];
f=px4_joint_response_cost(t,r,y,windows,1,.001);
offset=px4_joint_response_cost(t,r,y+.3,windows,1,.001);
ring=px4_joint_response_cost(t,r,y+.4*sin(2*pi*12*t),windows,1,.001);
control=px4_joint_response_cost(t,r,y,windows,1,.08);
scaled=px4_joint_response_cost(t,2*r,2*y,windows,2,.001);
assert(f.Usable&&offset.Score>f.Score&&ring.Score>f.Score&&control.Score>f.Score);
assert(~offset.Usable&&~ring.Usable&&~control.Usable&&abs(f.Score-scaled.Score)<1e-12);
fprintf('JOINT_UNIT_PASS: Default independence, all bounds, fixed K, low-I probe, replay, coupled valley, infeasible probes and response scoring.\n');
