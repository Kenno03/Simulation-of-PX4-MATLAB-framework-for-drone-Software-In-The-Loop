function files=plot_gain_evolution(Session,out)
%PLOT_GAIN_EVOLUTION Report figure: the PID gains flown in every tuning flight, relative to PX4 Default.
%   files = plot_gain_evolution(Session) writes GainEvolution_CB and GainEvolution_RG (PDF and PNG)
%   into Session.Folder; plot_gain_evolution(Session,out) writes them into OUT. Session is a
%   run_px4_guide_tuning session (guide or RMSE method; load(fullfile(folder,'session.mat'))).
%   One tile per loop; x = flight number, y = gain / PX4 Default gain. Lines: gains flown while
%   searching; x: robustness probes (gains x1.3, RMSE method); large outlined markers: the selected
%   gains in the final check. _CB is colour-blind safe (viridis colours plus line styles), _RG uses
%   standard red/green/blue colours. Gains whose Default is 0 are left out (ratio undefined).
%   px4_guide_save calls it when a session ends.
if nargin < 2 || isempty(out), out = char(Session.Folder); end
s = Session; F = s.Flights; def = s.StartGains;
files = strings(1,0);F = F(F.Status == "VALID", :);
loops = ["rollrate","pitchrate","yawrate","rollattitude","pitchattitude","yawattitude", ...
         "horizontalvelocity","verticalvelocity","horizontalposition","verticalposition"];
titles = ["Roll rate","Pitch rate","Yaw rate","Roll attitude","Pitch attitude","Yaw attitude", ...
          "Horizontal velocity","Vertical velocity","Horizontal position","Vertical position"];
finalStart = min(F.Flight(F.Purpose == "final check"));
if isempty(finalStart), finalStart = max(F.Flight) + 1; end   % session ended before the final check
pal.CB.col = [68 1 84; 49 104 142; 53 183 121; 222 190 0] / 255; pal.CB.ls = {'-','--','-.',':'}; pal.CB.mk = 'osd^';
pal.RG.col = [220 30 30; 30 160 30; 30 80 220; 240 140 0] / 255; pal.RG.ls = {'-','-','-','-'};  pal.RG.mk = 'oooo';
for name = ["CB", "RG"]
    P = pal.(name);
    fig = figure('Visible','off','Color','w','Units','centimeters','Position',[2 2 17 13]);
    try, theme(fig, 'light'); catch, end
    tl = tiledlayout(fig, 3, 4, 'Padding','tight', 'TileSpacing','compact');
    for i = 1:numel(loops)
        ax = nexttile(tl); hold(ax,'on'); grid(ax,'on'); box(ax,'on');
        R = F(F.Loop == loops(i), :);
        if isempty(R), axis(ax, 'off'); title(ax, titles(i) + " (not tuned)", 'FontWeight','normal', 'FontSize', 8.5); continue, end
        [names, vals] = parseGains(R.Gains);
        keep = arrayfun(@(n) isfield(def, n) && def.(n) ~= 0, names);
        names = names(keep); vals = vals(:, keep);
        final = R.Purpose == "final check";
        probe = startsWith(R.Purpose, "robustness probe");          % RMSE method: gains x1.3, not a candidate
        tuning = ~final & ~probe;
        for j = 1:numel(names)
            c = 1 + mod(j-1, 4); r = vals(:,j) / def.(names(j));
            plot(ax, R.Flight(tuning), r(tuning), [P.ls{c} P.mk(c)], 'Color', P.col(c,:), ...
                'MarkerFaceColor', P.col(c,:), 'MarkerSize', 3, 'LineWidth', 1.1, ...
                'DisplayName', shortName(names(j)));
            plot(ax, R.Flight(probe), r(probe), 'x', 'Color', P.col(c,:), 'MarkerSize', 5, 'LineWidth', 1, ...
                'HandleVisibility', 'off');
            plot(ax, R.Flight(final), r(final), P.mk(c), 'Color', 'k', 'MarkerFaceColor', P.col(c,:), ...
                'MarkerSize', 6, 'LineWidth', 0.8, 'HandleVisibility', 'off');
        end
        yline(ax, 1, ':', 'Color', [0.4 0.4 0.4], 'HandleVisibility', 'off');
        xline(ax, finalStart - 0.5, '-', 'Color', [0.6 0.6 0.6], 'HandleVisibility', 'off');
        xlim(ax, [0.5, max(F.Flight) + 0.5]);
        title(ax, titles(i), 'FontWeight','normal', 'FontSize', 8.5);
        set(ax, 'FontSize', 7);
        if mod(i-1, 4) == 0, ylabel(ax, 'Gain / PX4 Default'); end
        if i > 6, xlabel(ax, 'Flight number'); end
        lg = legend(ax, 'Location','northoutside', 'FontSize', 6.5, 'Box','off', 'Orientation','horizontal');
        lg.ItemTokenSize = [14 8];                                       % legend above the axes, never over data
        yl = ylim(ax); ylim(ax, [min(yl(1), 0.95), max(yl(2), 1.05) + 0.05 * diff(yl)]);
    end
    ax = nexttile(tl); axis(ax, 'off');
    note = {'Gain flown in each tuning flight,', 'relative to the PX4 Default gain.', ...
        'K, P, I, D: the loop''s gains.', '', 'Dotted line: Default (ratio 1).', ...
        sprintf('Grey line: start of the final check (flight %d);', finalStart), ...
        'large outlined markers = selected gains.'};
    if any(startsWith(F.Purpose, "robustness probe")), note{end+1} = 'x: robustness probe (gains x1.3).'; end
    text(ax, 0, 0.6, note, 'FontSize', 7, 'VerticalAlignment', 'middle');
    base = fullfile(out, "GainEvolution_" + name); files(end+1) = base + ".pdf"; files(end+1) = base + ".png"; %#ok<AGROW>
    exportgraphics(fig, base + ".pdf", 'ContentType', 'vector');
    exportgraphics(fig, base + ".png", 'Resolution', 300); close(fig);
end
end

function s = shortName(n)
% MC_ROLLRATE_K -> K, MPC_XY_VEL_P_ACC -> P, MPC_Z_P -> P (the tile title names the loop).
p = split(erase(n, "_ACC"), "_"); s = p(end);
end

function [names, vals] = parseGains(g)
% Parse "NAME=value NAME=value" strings (one per flight) into a flights x gains matrix.
tok = regexp(char(g(1)), '(\w+)=([-\d.eE+]+)', 'tokens');
names = string(cellfun(@(t) t{1}, tok, 'UniformOutput', false));
vals = NaN(numel(g), numel(names));
for k = 1:numel(g)
    t = regexp(char(g(k)), '(\w+)=([-\d.eE+]+)', 'tokens');
    for m = 1:numel(t), vals(k, names == t{m}{1}) = str2double(t{m}{2}); end
end
end
