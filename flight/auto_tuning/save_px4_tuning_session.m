function Session=save_px4_tuning_session(Session)
%SAVE_PX4_TUNING_SESSION Save a single-loop tuning session (checkpoint after every trial).
%   Session = save_px4_tuning_session(Session) writes session.mat, the results and gain-history CSV
%   files and the gain-history figure (FIG and PNG) to Session.Folder. Reload with
%   load_px4_tuning_session.
% Canonical, reloadable MATLAB artifacts; all files are in one session folder.
folder=char(Session.Folder);
if ~isempty(Session.GainHistory)
    parameters=unique(Session.GainHistory.Parameter,'stable');
    n=numel(parameters);
    columns=min(5,ceil(sqrt(n)));rows=ceil(n/columns);
    f=figure('Visible','off','Color','w', ...
        'Position',[100 100 max(1100,360*columns) max(780,230*rows)]);
    guard=onCleanup(@()close(f)); %#ok<NASGU>
    layout=tiledlayout(f,rows,columns,'Padding','compact','TileSpacing','compact');
    for parameter=parameters'
        ax=nexttile(layout);h=Session.GainHistory(Session.GainHistory.Parameter==parameter,:);
        plot(ax,h.Run,h.AcceptedValue,'-o','Color',[0.08 0.45 0.75],'LineWidth',1.8,'DisplayName','Accepted / retained');hold(ax,'on');
        plot(ax,h.Run,h.TrialValue,'x','Color',[0.65 0.4 0.15],'MarkerSize',8,'DisplayName','Trial value');
        plot(ax,h.Run(1),h.AcceptedValue(1),'s','MarkerFaceColor',[0.1 0.65 0.4], ...
            'Color',[0.1 0.5 0.3],'DisplayName','Start');
        plot(ax,h.Run(end),h.AcceptedValue(end),'d','MarkerFaceColor',[0.5 0.2 0.7], ...
            'Color',[0.5 0.2 0.7],'DisplayName','End');
        title(ax,strrep(parameter,'_','\_'));xlabel(ax,'Trial (0=start)');ylabel(ax,'Gain value');
        grid(ax,'on');ytickformat(ax,'%.5g');
        if parameter==parameters(1),legend(ax,'Location','best','FontSize',8);end
    end
    title(layout,'Gain evolution: starting, attempted, accepted and final values', ...
        'Color',[0.12 0.18 0.25]);
    style(f);savefig(f,fullfile(folder,'GainHistory.fig'));
    exportgraphics(f,fullfile(folder,'GainHistory.png'),'Resolution',140,'BackgroundColor','white');
    Session.FigureFiles=unique([Session.FigureFiles;"GainHistory.fig"],'stable');
    clear guard;
end
if ~isempty(Session.Results)
    f=figure('Visible','off','Color','w','Position',[100 100 1050 500]);
    guard=onCleanup(@()close(f)); %#ok<NASGU>
    ax=axes(f);T=Session.Results;valid=isfinite(T.Score);
    plot(ax,T.Run(valid),T.Score(valid),'o-','DisplayName','Valid trial score');hold(ax,'on');
    best=isfinite(T.BestScore);plot(ax,T.Run(best),T.BestScore(best),'-','LineWidth',2,'DisplayName','Best accepted score');
    if ~any(valid)
        text(ax,0.05,0.55,'No valid excitation/response score in this session.', ...
            'Units','normalized','FontSize',12);
    end
    xlabel(ax,'Session trial');ylabel(ax,'Normalized score (lower is better)');
    title(ax,'Session scores: compare within each controller stage');grid(ax,'on');legend(ax,'Location','best');
    style(f);savefig(f,fullfile(folder,'ScoreHistory.fig'));
    exportgraphics(f,fullfile(folder,'ScoreHistory.png'),'Resolution',140,'BackgroundColor','white');
    Session.FigureFiles=unique([Session.FigureFiles;"ScoreHistory.fig"],'stable');
    clear guard;
end
Results=Session.Results;GainHistory=Session.GainHistory;StageResults=Session.StageResults;
StartGains=Session.StartGains;BestGains=Session.BestGains;Settings=Session.Config;
save(fullfile(folder,'results.mat'),'Results','GainHistory','StageResults','StartGains','BestGains','Settings','-v7.3');
writetable(Results,fullfile(folder,'results.csv'));
writetable(GainHistory,fullfile(folder,'gain_history.csv'));
% The MAT checkpoint retains exact gain structs; CSV also exposes each field.
if isfield(Session,'JointHistory')&&~isempty(Session.JointHistory)
    JointHistory=Session.JointHistory;
    flat=removevars(JointHistory,'GainSet');
    for name=px4_controller_gain_names()
        flat.(name)=cellfun(@(g)double(g.(name)),JointHistory.GainSet);
    end
    Quality=Session.Quality;
    save(fullfile(folder,'joint_search.mat'),'JointHistory','Quality','-v7.3');
    writetable(flat,fullfile(folder,'joint_search.csv'));
    if ~isempty(Quality),writetable(Quality,fullfile(folder,'quality.csv'));end
end
temporary=fullfile(folder,'session_checkpoint.mat');
save(temporary,'Session','-v7.3');
movefile(temporary,fullfile(folder,'session.mat'),'f');
end
function style(f)
dark=[0.12 0.18 0.25];
set(findall(f,'Type','axes'),'Color','white','XColor',dark,'YColor',dark,'GridColor',[0.7 0.75 0.8]);
set(findall(f,'Type','text'),'Color',dark);
set(findall(f,'Type','legend'),'Color','white','TextColor',dark);
end
