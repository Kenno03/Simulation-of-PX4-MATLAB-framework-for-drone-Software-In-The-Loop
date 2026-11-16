function Session=px4_guide_save(Session)
%PX4_GUIDE_SAVE Save a multi-axis tuning session (MAT, CSV, decisions log, gain figures).
%   Session = px4_guide_save(Session) writes session.mat, guide_flights.csv, guide_final_check.csv,
%   guide_decisions.txt and the quick GuideGainHistory figure (every checkpoint) to Session.Folder.
%   When the session has ended it also writes the report figures GainEvolution_CB/RG
%   (plot_gain_evolution). Called after every flight by run_px4_guide_tuning.
folder=char(Session.Folder);
if ~isempty(Session.Flights),writetable(Session.Flights,fullfile(folder,'guide_flights.csv'));end
if ~isempty(Session.Final),writetable(Session.Final,fullfile(folder,'guide_final_check.csv'));end
fid=fopen(fullfile(folder,'guide_decisions.txt'),'wt');
if fid>0,fprintf(fid,'%s\n',Session.Decisions);fclose(fid);end
if ~isempty(Session.Flights)
    T=Session.Flights;loops=unique(T.Loop,'stable');
    f=figure('Visible','off','Color','w','Position',[100 100 1300 260*ceil(numel(loops)/3)+80]);
    guard=onCleanup(@()close(f)); %#ok<NASGU>
    layout=tiledlayout(f,ceil(numel(loops)/3),3,'Padding','compact','TileSpacing','compact');
    for loop=loops'
        ax=nexttile(layout);t=T(T.Loop==loop,:);hold(ax,'on');
        names=regexp(t.Gains(1),'(\w+)=','tokens');names=string([names{:}]);
        for n=names
            v=cellfun(@(s)str2double(regexp(s,n+"=([-\d\.eE+]+)",'tokens','once')),cellstr(t.Gains));
            plot(ax,t.Flight,v./max(abs(v(1)),eps),'-o','DisplayName',strrep(n,'_','\_'));
        end
        yyaxis(ax,'right');plot(ax,t.Flight,t.Score,'k:x','DisplayName','score');ylabel(ax,'Score');
        yyaxis(ax,'left');ylabel(ax,'Gain / start value');xlabel(ax,'Flight');grid(ax,'on');
        title(ax,loop,'Interpreter','none');legend(ax,'Location','best','FontSize',7);
    end
    savefig(f,fullfile(folder,'GuideGainHistory.fig'));
    exportgraphics(f,fullfile(folder,'GuideGainHistory.png'),'Resolution',130,'BackgroundColor','white');
end
temporary=fullfile(folder,'session_checkpoint.mat');save(temporary,'Session','-v7.3');
movefile(temporary,fullfile(folder,'session.mat'),'f');
% report figures (both palettes) once the session has ended
if ~ismember(Session.Status,["INITIALIZING","RUNNING"])&&~isempty(Session.Flights)
    try,plot_gain_evolution(Session,folder);catch ME,fprintf(2,'Gain-evolution figure: %s\n',ME.message);end
end
end
