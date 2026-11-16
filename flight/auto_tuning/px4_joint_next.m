function [search,candidate,label,changed]=px4_joint_next(search,best,bestScore)
%PX4_JOINT_NEXT Next gain vector of the joint finite-difference search (single-loop tuning).
%   [search,candidate,label,changed] = px4_joint_next(search,best,bestScore) returns the next gains to
%   fly: a +/- sensitivity probe per coordinate around the accepted centre, then one combined downhill
%   step (bounded; the radius halves after a cycle without improvement). LABEL describes the proposal
%   and CHANGED lists the gains that differ from BEST.
% Replay an unresolved proposal after an execution-only continuation.
if ~isempty(fieldnames(search.Pending))&&~search.Pending.Resolved
    candidate=apply(best,search,search.Pending.X);
    label=search.Pending.Label;changed=search.Pending.Changed;return;
end
search.CycleComplete=false;
center=arrayfun(@(n)double(best.(n)),search.Names)./search.Scale;
for guard=1:100
    switch search.Phase
        case "seed"
            if search.SeedIndex==1
                x=search.Initial*.8;label="Joint cold-start probe";
            elseif search.SeedIndex==2
                x=search.Initial*.9;
                integral=find(endsWith(search.Names,["_I","_I_ACC"]));
                x(integral)=search.Lower(integral);
                zero=search.Initial==0;x(zero)=.2;
                label="Joint low-integral probe";
            else
                search.Phase="cycle";continue;
            end
            search.SeedIndex=search.SeedIndex+1;
            kind="seed";index=0;
        case "cycle"
            search.Center=center;search.CenterScore=bestScore;
            search.ProbeScores=nan(1,2*numel(center));
            search.ProbeX=repmat(center,2*numel(center),1);
            for k=1:numel(center)
                search.ProbeX(2*k-1,k)=min(search.Upper(k),center(k)+search.Radius);
                search.ProbeX(2*k,k)=max(search.Lower(k),center(k)-search.Radius);
            end
            search.ProbeIndex=1;search.CycleImproved=false;search.Phase="probe";continue;
        case "probe"
            if search.ProbeIndex>size(search.ProbeX,1)
                search.Phase="joint";continue;
            end
            index=search.ProbeIndex;search.ProbeIndex=index+1;
            x=search.ProbeX(index,:);kind="probe";
            label="Sensitivity "+search.Names(ceil(index/2));
            if all(x==search.Center)
                search.ProbeScores(index)=search.CenterScore;continue;
            end
        case "joint"
            gradient=zeros(size(search.Center));
            for k=1:numel(gradient)
                pos=2*k-1;neg=2*k;xp=search.ProbeX(pos,k);xn=search.ProbeX(neg,k);
                jp=search.ProbeScores(pos);jn=search.ProbeScores(neg);
                if isfinite(jp)&&isfinite(jn)&&xp>xn
                    gradient(k)=(jp-jn)/(xp-xn);
                elseif isfinite(jp)&&xp>search.Center(k)
                    gradient(k)=(jp-search.CenterScore)/(xp-search.Center(k));
                elseif isfinite(jn)&&xn<search.Center(k)
                    gradient(k)=(search.CenterScore-jn)/(search.Center(k)-xn);
                end
            end
            if any(abs(gradient)>1e-9)
                direction=-gradient/max(abs(gradient));
                label="Joint gradient update";
            else
                % A flat/noisy local estimate still gets a coupled direction.
                direction=sign(cos((1:numel(gradient))*(search.CompletedCycles+1)));
                label="Joint flat-region probe";
            end
            x=search.Center+search.Radius*direction;kind="joint";index=0;
        otherwise,error('Unknown joint-search phase.');
    end
    x=min(max(x,search.Lower),search.Upper);
    if all(abs(x-center)<1e-12)&&kind=="seed",continue;end
    changed=search.Names(abs(x-center)>1e-12);
    search.Pending=struct('X',x,'Kind',kind,'Index',index,'Label',label, ...
        'Changed',changed,'Resolved',false);
    candidate=apply(best,search,x);return;
end
error('No joint proposal can move within the configured bounds.');
end
function tune=apply(tune,search,x)
for k=1:numel(search.Names),tune.(search.Names(k))=x(k)*search.Scale(k);end
end
