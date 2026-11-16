function T=compare_missions(results,options)
%COMPARE_MISSIONS Compare several run_mission results: table, bar charts and path plots.
%   T = compare_missions([A B C]) returns one row per result with the mean and standard deviation of
%   every score_mission metric, and writes report figures next to the first result:
%     MissionBars_CB / _RG    bar charts (mean, std error bars, individual flights as dots)
%     MissionPaths_CB / _RG   top view, tracking error and heading error of each result's first flight
%   _CB uses a colour-blind-safe palette (viridis colours plus line styles); _RG uses standard
%   red/green/blue colours (not colour-blind safe). Each figure is written as PDF (vector) and PNG.
%   T = compare_missions(results,OutputFolder=folder,Tag="Figure8") sets the folder and a file-name tag.
arguments
    results (1,:) struct
    options.OutputFolder (1,1) string = ""
    options.Tag (1,1) string = ""
end
out=options.OutputFolder;if out=="",out=fileparts(results(1).Folder);end
if ~isfolder(out),mkdir(out);end
names=string({results.Name});n=numel(results);
metrics=["RMS3D_cm","PathDev_RMS_cm","Lag_s","AltRMS_cm","HeadingRMS_deg","MaxTilt_deg","MotorRate_rms","Score"];
titles=["Tracking error RMS [cm]","Distance from path RMS [cm]","Lag [s]","Altitude error RMS [cm]", ...
    "Heading error RMS [deg]","Max tilt [deg]","Motor-command rate [1/s]","Composite score"];
T=table();
for i=1:n
    F=results(i).Flights;ok=startsWith(F.Status,"OK");row=table(names(i),nnz(ok),'VariableNames',{'Name','Flights'});
    for m=metrics,v=F.(m)(ok);row.(m+"_mean")=mean(v);row.(m+"_std")=std(v);end
    T=[T;row]; %#ok<AGROW>
end
writetable(T,fullfile(out,"MissionComparison"+options.Tag+".csv"));disp(T);
cb=[68 1 84;59 82 139;33 145 140;94 201 98;253 231 37;180 180 180]/255;
rg=[220 30 30;30 160 30;30 80 220;240 140 0;150 0 150;90 90 90]/255;
pal.CB=struct('col',cb,'ls',{{'-','--','-.',':','-','--'}});
pal.RG=struct('col',rg,'ls',{{'-','-','-','-','-','-'}});
assert(n<=size(cb,1),'At most %d results per comparison.',size(cb,1));
% colour-blind palette: spread the colours over the viridis range (purple -> yellow) for any n
pick=round(linspace(1,5,max(n,2)));pick=pick(1:n);if n==1,pick=1;end
pal.CB.col=cb(pick,:);pal.CB.ls=pal.CB.ls(1:n);
for name=["CB","RG"]
    P=pal.(name);
    % bar charts
    fig=figure('Visible','off','Color','w','Units','centimeters','Position',[2 2 17 10]);try,theme(fig,'light');catch,end
    tl=tiledlayout(fig,2,3,'Padding','tight','TileSpacing','compact');
    for j=[1 2 4 5 7 8]
        ax=nexttile(tl);hold(ax,'on');grid(ax,'on');box(ax,'on');m=metrics(j);
        for i=1:n
            bar(ax,i,T.(m+"_mean")(i),0.65,'FaceColor',P.col(i,:),'EdgeColor','k','LineWidth',0.5);
            errorbar(ax,i,T.(m+"_mean")(i),T.(m+"_std")(i),'k','LineWidth',1,'CapSize',6);
            F=results(i).Flights;v=F.(m)(startsWith(F.Status,"OK"));
            scatter(ax,i+linspace(-0.12,0.12,numel(v))',v,12,'k','filled','MarkerFaceAlpha',0.6);
        end
        set(ax,'XTick',1:n,'XTickLabel',names,'FontSize',7);ax.XTickLabelRotation=25;
        title(ax,titles(j),'FontWeight','normal','FontSize',8.5);
    end
    save2(fig,fullfile(out,"MissionBars"+options.Tag+"_"+name));
    % paths of the first valid flight of each result
    fig=figure('Visible','off','Color','w','Units','centimeters','Position',[2 2 16 8.5]);try,theme(fig,'light');catch,end
    tl=tiledlayout(fig,1,3,'Padding','tight','TileSpacing','compact');
    ax1=nexttile(tl);hold(ax1,'on');ax2=nexttile(tl);hold(ax2,'on');ax3=nexttile(tl);hold(ax3,'on');
    h=gobjects(0);hRef=[];
    for i=1:n
        k=find(~cellfun(@isempty,results(i).Scores),1);if isempty(k),continue,end
        M=results(i).Scores{k};w=M.window;
        if isempty(hRef),hRef=plot(ax1,M.path(:,2),M.path(:,1),'-','LineWidth',2.6,'Color',[0.75 0.75 0.75]);end
        c=M.t>=w(1)-1&M.t<=w(2)+4;lw=1.3+0.5*strcmp(P.ls{i},':');
        h(end+1)=plot(ax1,M.p(c,2),M.p(c,1),P.ls{i},'Color',P.col(i,:),'LineWidth',lw); %#ok<AGROW>
        c=M.t>=w(1)-2&M.t<=w(2)+15;
        plot(ax2,M.t(c)-w(1),100*vecnorm(M.p(c,:)-M.ref(c,:),2,2),P.ls{i},'Color',P.col(i,:),'LineWidth',lw);
        plot(ax3,M.t(c)-w(1),M.eyaw(c),P.ls{i},'Color',P.col(i,:),'LineWidth',lw);
    end
    axis(ax1,'equal');xlabel(ax1,'East [m]');ylabel(ax1,'North [m]');title(ax1,'Top view','FontWeight','normal','FontSize',9);
    xlabel(ax2,'Time since path start [s]');ylabel(ax2,'Position error [cm]');title(ax2,'Tracking error','FontWeight','normal','FontSize',9);
    xlabel(ax3,'Time since path start [s]');ylabel(ax3,'Heading error [deg]');title(ax3,'Heading error','FontWeight','normal','FontSize',9);
    for ax=[ax1 ax2 ax3],grid(ax,'on');box(ax,'on');set(ax,'FontSize',8);end
    if ~isempty(hRef)
        lg=legend(ax3,[hRef h],['Reference path',cellstr(names(1:numel(h)))],'NumColumns',min(n+1,5),'Box','off','FontSize',8);
        lg.Layout.Tile='south';
    end
    save2(fig,fullfile(out,"MissionPaths"+options.Tag+"_"+name));
end
end

function save2(fig,base)
exportgraphics(fig,base+".pdf",'ContentType','vector');exportgraphics(fig,base+".png",'Resolution',300);close(fig);
end
