function T = scenarioSummaryHRC(folder, opts)
% Table and figure of ALL recorded scenarios, made from the saved files only
% (no new experiment). Every results\...\scenario_*.mat below folder is read
% and its metrics are recomputed (scenarioMetricsHRC).
%
% T = scenarioSummaryHRC('results')
% writes results\scenario_summary.csv (one row per run, one column per metric)
% and results\scenario_summary.pdf / .png: share of time at normal / slow /
% stopped / protective stop for every run (IEEE style).
% opts.width (in, default 3.5), opts.height (default 2.6)
if nargin < 1 || isempty(folder), folder = 'results'; end
if nargin < 2, opts = struct(); end
L = dir(fullfile(folder, '**', 'scenario_*.mat'));
if isempty(L)
    error('no scenario_*.mat found under %s', folder);
end
rows = {};
for i = 1:numel(L)
    S = load(fullfile(L(i).folder, L(i).name));
    if ~isfield(S,'rec'), continue; end
    m = scenarioMetricsHRC(S.rec, S.prm);
    if isfield(m, 'duration_s'), rows{end+1} = m; end %#ok<AGROW>
end
T = struct2table([rows{:}]);
T = sortrows(T, {'mode','name'});
writetable(T, fullfile(folder, 'scenario_summary.csv'));
fprintf('%d runs -> %s\n', height(T), fullfile(folder, 'scenario_summary.csv'));
% ---------------- figure: time share of the robot motion per run
W = 3.5; H = 2.6;
if isfield(opts,'width'), W = opts.width; end
if isfield(opts,'height'), H = opts.height; end
fn = 'Times New Roman'; fs = 8;
fig = figure('Color','w','Units','inches','Position',[1 1 W H],'Name','Scenario summary');
ax = axes(fig); hold(ax,'on');
Y = [T.share_normal_pct T.share_slow_pct T.share_stopped_pct T.share_protective_stop_pct];
if size(Y,1)==1, Y = [Y; NaN(1,4)]; end            % one run: keep the stacked layout
b = barh(ax, Y, 'stacked', 'BarWidth', 0.7, 'EdgeColor', 'none');
col = {[0 114 178]/255, [86 180 233]/255, [230 159 0]/255, [204 37 41]/255};
for k = 1:4, b(k).FaceColor = col{k}; end
lab = strrep(T.name, '_', ' ');
set(ax,'YTick',1:height(T),'YTickLabel',lab,'YDir','reverse','XLim',[0 100],'YLim',[0.4 height(T)+0.6], ...
    'FontName',fn,'FontSize',fs,'LineWidth',0.5,'Box','on','TickLabelInterpreter','none');
xlabel(ax,'Share of time (%)');
lg = legend(ax, b, {'Normal speed','Slow','Stopped','Protective stop'}, 'Location','southoutside', ...
    'Orientation','horizontal','Box','off','NumColumns',4,'FontName',fn,'FontSize',fs-1);
lg.ItemTokenSize = [10 8];
base = fullfile(folder, 'scenario_summary');
try
    exportgraphics(fig, [base '.pdf'], 'ContentType','vector');
    exportgraphics(fig, [base '.png'], 'Resolution', 600);
catch err
    warning('scenarioSummaryHRC:export','export failed (%s)', err.message);
end
end
