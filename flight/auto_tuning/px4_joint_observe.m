function search=px4_joint_observe(search,score,valid,accepted)
%PX4_JOINT_OBSERVE Record the score of the last proposal in the joint finite-difference search.
%   search = px4_joint_observe(search,score,valid,accepted) stores the result of the pending probe or
%   combined step and updates the cycle bookkeeping used by px4_joint_next.
% Invalid gain probes are infeasible; execution faults are not observed here.
assert(~isempty(fieldnames(search.Pending))&&~search.Pending.Resolved);
if ~valid,score=Inf;end
if search.Pending.Kind=="probe"
    search.ProbeScores(search.Pending.Index)=score;
end
search.CycleImproved=search.CycleImproved||accepted;
if search.Pending.Kind=="joint"
    search.CompletedCycles=search.CompletedCycles+1;search.CycleComplete=true;
    if ~search.CycleImproved,search.Radius=max(.01,search.Radius/2);end
    search.Phase="cycle";
end
search.Pending.Resolved=true;
end
