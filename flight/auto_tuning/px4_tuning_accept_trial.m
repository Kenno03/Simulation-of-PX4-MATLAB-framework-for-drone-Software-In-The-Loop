function [state,accepted,improvement]=px4_tuning_accept_trial(state,tune,score,valid,tolerance)
%PX4_TUNING_ACCEPT_TRIAL Accept or reject one single-loop trial (run_px4_tuning).
%   [state,accepted,improvement] = px4_tuning_accept_trial(state,tune,score,valid,tolerance) accepts
%   TUNE when the trial is valid, its gains differ from the accepted ones and SCORE improves the best
%   score by at least TOLERANCE (relative). It updates the best score/gains and the stagnation count.
% Failed flights are diagnosed separately and do not advance stagnation.
accepted=false;
improvement=NaN;
if ~valid
    return;
end
assert(isfinite(score),'A valid trial requires a finite score.');

if ~state.HasBaseline
    state.BestScore=score;
    state.BestTune=tune;
    state.HasBaseline=true;
    state.StagnantValidTrials=0;
    accepted=true;
    improvement=Inf;
else
    % Relative score improvement avoids an absolute cutoff on tiny gains.
    improvement=(state.BestScore-score)/max(abs(state.BestScore),eps);
    % A repeat at identical gains is measurement evidence, not a new gain
    % improvement. Do not reset convergence from a lucky repeated score.
    if ~isequaln(tune,state.BestTune) && improvement>=tolerance && score<state.BestScore
        state.BestScore=score;
        state.BestTune=tune;
        state.StagnantValidTrials=0;
        accepted=true;
    else
        state.StagnantValidTrials=state.StagnantValidTrials+1;
    end
end
end
