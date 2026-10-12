function plotScenariosHRC(files, opts)
% JOURNAL FIGURES OF RECORDED SCENARIOS (IEEE Transactions style), from the saved .mat files.
%
% plotScenariosHRC()                 the last 4 recorded scenarios (results\scenario_*\scenario_*.mat)
% plotScenariosHRC(files)            cell array of scenario .mat files (plotted in this order)
% plotScenariosHRC(files, opts)      opts.titles  column titles (default from the task mode)
%                                    opts.out     output folder (default results\figures_<date>)
%
% Writes into opts.out:
%   fig_combined.pdf / .png / .tif / .fig   ONE figure, IEEE double column (7.16 in): one column per
%       scenario, shared rows (a) fatigue signals + estimate confidence (background: sustained fatigue
%       level), (b) time looking away + 2 s limit, (c) hand-tool distance, (d) commanded / measured joint
%       speed (protective stops shaded), (e) cause of a speed reduction, (f) robot action (colour =
%       motion); events (tool request, rest warning, rest, ...) above the columns; one legend below
%   fig_scenario_<k>.pdf / .png / .tif / .fig   the same rows for each scenario, IEEE single column (3.5 in)
%   scenario_summary.csv                    numbers for the text: duration, share of time at normal / slow /
%       stopped / protective stop, mean commanded speed, min / median hand-tool distance, protective
%       stops, look-aways >= 2 s, highest fatigue level, time per speed-reduction cause
% Style: Times New Roman 8 pt, 0.5 pt axes, Okabe-Ito colours plus line styles (readable in gray scale),
% vector PDF, 600 dpi PNG / TIFF. Every epoch is drawn with its real duration (time stamps of the recording).
here = fileparts(mfilename('fullpath'));
if nargin < 1 || isempty(files)
    d = dir(fullfile(here, 'results', 'scenario_*', 'scenario_*.mat'));
    [~, i] = sort([d.datenum]);
    d = d(i(max(1, end-3):end));
    files = fullfile({d.folder}, {d.name});
end
if nargin < 2, opts = struct(); end
out = getOpt(opts, 'out', fullfile(here, 'results', ['figures_' char(datetime('now','Format','yyyyMMdd_HHmmss'))]));
if ~exist(out, 'dir'), mkdir(out); end
% ---------------- data of every scenario
E = cell(1, numel(files));
for k = 1:numel(files)
    L = load(files{k}, 'rec', 'prm');
    E{k} = scenarioData(L.rec, L.prm);
end
roman = {'I','II','III','IV','V','VI','VII','VIII'};
titles = getOpt(opts, 'titles', {});
if isempty(titles)
    titles = cell(1, numel(E));
    for k = 1:numel(E), titles{k} = sprintf('(%s) %s', roman{k}, E{k}.modeTitle); end
end
% rows of the categorical panels: the same in every column (union over the scenarios)
S.causes = unique(cell2mat(cellfun(@(e) e.cause(e.cause > 0)', E, 'UniformOutput', false)));
S.actions = actionRows(cell2mat(cellfun(@(e) e.aPlot(:)', E, 'UniformOutput', false)), ...
    cell2mat(cellfun(@(e) e.aN(:)', E, 'UniformOutput', false)));
S.distTop = max(1.0, 1.1 * max(cellfun(@(e) max(e.dist, [], 'omitnan'), E)));
S.speedTop = 35;
S.awayTop = max(4.5, 1.15 * max(cellfun(@(e) max([e.away; 0], [], 'omitnan'), E)));
S.prm = E{1}.prm;
% ---------------- combined figure (double column)
W = 7.16; H = 8.4;
f = figure('Color','w','Units','inches','Position',[0.5 0.5 W H],'Visible','off', ...
    'PaperUnits','inches','PaperPosition',[0 0 W H],'PaperSize',[W H]);
nc = numel(E);
left = 1.12; right = 0.06; gapC = 0.12; top = 0.78; bottom = 0.95;
rowsW = [1.05 0.62 0.80 0.95 0.30 + 0.13*max(1, numel(S.causes)) 0.30 + 0.13*numel(S.actions)];
gapR = 0.07;
colW = (W - left - right - gapC*(nc-1)) / nc;
rowH = (H - top - bottom - gapR*(numel(rowsW)-1)) * rowsW / sum(rowsW);
AX = gobjects(numel(rowsW), nc);
for k = 1:nc
    x0 = left + (k-1) * (colW + gapC);
    AX(:, k) = drawScenario(f, E{k}, S, x0, colW, H - top, rowH, gapR, k == 1, titles{k});
end
addLegend(f, W, 0.04, 0.55, S);
exportAll(f, fullfile(out, 'fig_combined'));
% ---------------- one figure per scenario (single column)
for k = 1:nc
    S1 = S;                                      % categorical rows of THIS scenario only
    S1.causes = unique(E{k}.cause(E{k}.cause > 0))';
    S1.actions = actionRows(E{k}.aPlot, E{k}.aN);
    rw = [1.05 0.62 0.80 0.95 0.30 + 0.13*max(1, numel(S1.causes)) 0.30 + 0.13*numel(S1.actions)];
    W1 = 3.5; H1 = 7.4; legH = 1.30; bot = legH + 0.42;
    f1 = figure('Color','w','Units','inches','Position',[0.5 0.5 W1 H1],'Visible','off', ...
        'PaperUnits','inches','PaperPosition',[0 0 W1 H1],'PaperSize',[W1 H1]);
    rH = (H1 - 0.72 - bot - gapR*(numel(rw)-1)) * rw / sum(rw);
    drawScenario(f1, E{k}, S1, 1.12, W1 - 1.12 - 0.06, H1 - 0.72, rH, gapR, true, titles{k});
    addLegend(f1, W1, 0.03, legH, S1);
    exportAll(f1, fullfile(out, sprintf('fig_scenario_%d', k)));
end
% ---------------- numbers for the text
T = summaryTable(E, titles);
writetable(T, fullfile(out, 'scenario_summary.csv'));
disp(T);
fprintf('figures and table saved in %s\n', out);
end

% =========================================================================
function e = scenarioData(rec, prm)
D = rec.data; c = rec.cols; n = size(D, 1);
g = @(name) colOr(D, c, name, NaN(n, 1));
e.prm = prm; e.name = rec.name;
e.t = g('t') - D(1, c.t);
dt = diff(e.t); dtm = median(dt(dt > 0));
e.t1 = [e.t(2:end); e.t(end) + dtm];             % end of every epoch (real duration)
e.mode = g('mode');
modes = {'Solo','Assistive','Collaborative'};
um = unique(e.mode(isfinite(e.mode)));
e.modeTitle = strjoin(modes(um + 1), ' + ');
e.Fp = g('Fp'); e.Fm = g('Fm'); e.level = g('level'); e.conf = g('conf');
e.ms = g('mentalScore'); e.pf = g('physicalF'); e.away = g('awayS');
e.dist = g('dHand'); e.verified = g('verified');
e.cmd = g('cmdSpeed'); e.meas = g('jointSpeed');
e.aPlot = g('aF'); e.aN = g('aN'); e.motion = g('motion');
e.aFraw = colOr(D, c, 'aFraw', e.aPlot);
e.cause = g('cause'); e.cause(~isfinite(e.cause)) = 0;
e.ev = sessionEvents(D, c, e.t);
end

function ax = drawScenario(f, e, S, x0, w, yTop, rowH, gapR, showY, ttl)
fs = 8; fn = 'Times New Roman';
C = palette();
nr = numel(rowH);
ax = gobjects(nr, 1);
y = yTop;
for r = 1:nr
    y = y - rowH(r);
    ax(r) = axes(f, 'Units','inches','Position',[x0 y w rowH(r)]); %#ok<LAXES>
    hold(ax(r), 'on');
    y = y - gapR;
end
tEnd = e.t1(end);
prm = S.prm;
% ---- (a) fatigue sensor signals + confidence; background = sustained fatigue level
a = ax(1);
band(a, e, e.level == 1, [0 1.08], C.medBg);
band(a, e, e.level == 2, [0 1.08], C.highBg);
plot(a, [0 tEnd], [0.6 0.6], ':', 'Color', C.blue, 'LineWidth', 0.6);
plot(a, [0 tEnd], [0.7 0.7], ':', 'Color', C.verm, 'LineWidth', 0.6);
plot(a, mid(e), e.conf, '-', 'Color', C.gray, 'LineWidth', 0.7);
plot(a, mid(e), e.pf, '-', 'Color', C.blue, 'LineWidth', 1.0);
plot(a, mid(e), e.ms, '--', 'Color', C.verm, 'LineWidth', 1.0);
set(a, 'YLim', [0 1.08], 'YTick', 0:0.5:1);
lab(a, showY, {'Fatigue','(0-1)'});
% events above the column
for k = 1:numel(e.ev.t)
    for r = 1:nr, xline(ax(r), e.ev.t(k), ':', 'Color', [0.35 0.35 0.35], 'LineWidth', 0.6); end
end
row = zeros(size(e.ev.t));
for k = 2:numel(e.ev.t)
    if e.ev.t(k) - e.ev.t(k-1) < 0.22 * tEnd, row(k) = mod(row(k-1) + 1, 3); end
end
hIn = rowH(1);                                   % row (a) height [in]: event rows 0.11 in apart above it
for k = 1:numel(e.ev.t)
    text(a, e.ev.t(k) / tEnd, 1 + (0.03 + 0.11 * row(k)) / hIn, e.ev.name{k}, 'Units', 'normalized', ...
        'FontName', fn, 'FontSize', fs - 2, 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', ...
        'Clipping', 'off', 'Color', [0.2 0.2 0.2]);
end
text(a, 0.5, 1 + 0.40 / hIn, ttl, 'Units', 'normalized', 'FontName', fn, 'FontSize', fs + 1, ...
    'FontWeight', 'bold', 'HorizontalAlignment', 'center', 'VerticalAlignment', 'bottom', 'Clipping', 'off');
% ---- (b) attention: time looking away
a = ax(2);
plot(a, [0 tEnd], prm.distractionLimit * [1 1], '--', 'Color', C.red, 'LineWidth', 0.7);
plot(a, mid(e), e.away, '-', 'Color', C.green, 'LineWidth', 1.0);
set(a, 'YLim', [0 S.awayTop]);
lab(a, showY, {'Look-away','(s)'});
% ---- (c) hand-tool distance
a = ax(3);
band(a, e, e.verified == 0, [0 S.distTop], C.light);
plot(a, [0 tEnd], prm.dCollabMax * [1 1], '-.', 'Color', C.orange, 'LineWidth', 0.7);
plot(a, mid(e), e.dist, '-', 'Color', 'k', 'LineWidth', 0.9);
set(a, 'YLim', [0 S.distTop]);
lab(a, showY, {'Distance','(m)'});
% ---- (d) joint speed: commanded (decision) and measured (robot)
a = ax(4);
band(a, e, e.aFraw == 10, [0 S.speedTop], C.stopBg);
plot(a, mid(e), movmean(e.meas, 3, 'omitnan'), '-', 'Color', C.blue, 'LineWidth', 0.8);
[xs, ys] = steps(e, e.cmd);
plot(a, xs, ys, '-', 'Color', 'k', 'LineWidth', 1.1);
set(a, 'YLim', [0 S.speedTop], 'YTick', 0:10:30);
lab(a, showY, {'Speed','(deg/s)'});
% ---- (e) cause of a speed reduction
a = ax(5);
cn = causeNames(); cc = causeColours();
for k = 1:numel(S.causes)
    idx = find(e.cause == S.causes(k));
    segs(a, e, idx, k, cc(S.causes(k), :), 3.5);
end
set(a, 'YLim', [0.4 max(1, numel(S.causes)) + 0.6], 'YTick', 1:numel(S.causes), ...
    'YTickLabel', ifelse(showY, cn(S.causes), {}));

% ---- (f) robot action (colour = motion)
a = ax(6);
mc = motionColours(); lvls = [-1 0 1 2];
[~, rowA] = ismember(e.aPlot, S.actions);
[~, rowN] = ismember(e.aN, S.actions);
rowN = double(rowN); rowN(rowN == 0) = NaN;
[xn, yn] = steps(e, rowN);
plot(a, xn, yn, '-', 'Color', C.gray, 'LineWidth', 0.5);
for m = 1:4
    idx = find(e.motion == lvls(m) & rowA > 0);
    for j = idx(:)'
        plot(a, [e.t(j) e.t1(j)], rowA(j) * [1 1], '-', 'Color', mc(m, :), 'LineWidth', 3.5);
    end
end
an = actionNames();
set(a, 'YLim', [0.4 numel(S.actions) + 0.6], 'YTick', 1:numel(S.actions), ...
    'YTickLabel', ifelse(showY, an(S.actions + 1), {}));

xlabel(a, 'Time (s)', 'FontName', fn, 'FontSize', fs);
% ---- common style
pl = 'abcdef';
for r = 1:nr
    set(ax(r), 'XLim', [0 tEnd], 'FontName', fn, 'FontSize', fs, 'LineWidth', 0.5, 'Box', 'on', ...
        'TickDir', 'in', 'TickLength', [0.015 0.015], 'XGrid', 'on', 'YGrid', 'on', ...
        'GridLineStyle', ':', 'GridAlpha', 0.3, 'Layer', 'top');
    if r < nr, set(ax(r), 'XTickLabel', []); end
    if ~showY && r ~= 5 && r ~= 6, set(ax(r), 'YTickLabel', []); end
    if showY
        text(ax(r), -1.05 / w, 1.0, ['(' pl(r) ')'], 'Units', 'normalized', 'FontName', fn, 'FontSize', fs, ...
            'FontWeight', 'bold', 'HorizontalAlignment', 'left', 'VerticalAlignment', 'top');
    end
end
set([ax(5) ax(6)], 'FontSize', fs - 1);
end

function addLegend(f, W, y0, h, S)
% one legend for all panels, below the axes
fn = 'Times New Roman'; fs = 7;
C = palette(); mc = motionColours(); cc = causeColours(); cn = causeNames();
la = axes(f, 'Units','inches','Position',[0.05 y0 W - 0.1 h],'Visible','off'); hold(la, 'on');
hh = [plot(la, NaN, NaN, '-', 'Color', C.blue, 'LineWidth', 1.0), plot(la, NaN, NaN, '--', 'Color', C.verm, 'LineWidth', 1.0), ...
      plot(la, NaN, NaN, '-', 'Color', C.gray, 'LineWidth', 0.7), patch(la, NaN, NaN, C.medBg, 'EdgeColor', 'none'), ...
      patch(la, NaN, NaN, C.highBg, 'EdgeColor', 'none'), plot(la, NaN, NaN, '-', 'Color', C.green, 'LineWidth', 1.0), ...
      plot(la, NaN, NaN, '--', 'Color', C.red, 'LineWidth', 0.7), plot(la, NaN, NaN, '-', 'Color', 'k', 'LineWidth', 0.9), ...
      plot(la, NaN, NaN, '-.', 'Color', C.orange, 'LineWidth', 0.7), plot(la, NaN, NaN, '-', 'Color', 'k', 'LineWidth', 1.1), ...
      plot(la, NaN, NaN, '-', 'Color', C.blue, 'LineWidth', 0.8), patch(la, NaN, NaN, C.stopBg, 'EdgeColor', 'none'), ...
      plot(la, NaN, NaN, '-', 'Color', mc(4, :), 'LineWidth', 3.5), plot(la, NaN, NaN, '-', 'Color', mc(3, :), 'LineWidth', 3.5), ...
      plot(la, NaN, NaN, '-', 'Color', mc(2, :), 'LineWidth', 3.5), plot(la, NaN, NaN, '-', 'Color', mc(1, :), 'LineWidth', 3.5), ...
      plot(la, NaN, NaN, '-', 'Color', C.gray, 'LineWidth', 0.5)];
nm = {'Physical fatigue index f','Mental fatigue score','Estimate confidence','Fatigue level medium','Fatigue level high', ...
      'Time looking away',sprintf('Distraction limit %.0f s', S.prm.distractionLimit),'Hand-tool distance', ...
      sprintf('Collaborative distance %.2f m', S.prm.dCollabMax),'Commanded speed','Measured speed (3-sample mean)', ...
      'Protective stop','Normal speed','Slow','Stopped (hold / wait)','Protective stop (action)','Planner (nominal)'};
lg = legend(la, hh, nm, 'NumColumns', ceil(W / 1.9), 'Box', 'off', 'Location', 'north', ...
    'FontName', fn, 'FontSize', fs);
lg.ItemTokenSize = [12 6];
lg.Units = 'inches'; lg.Position = [0.05 y0 W - 0.1 h];
end

function band(a, e, mask, yl, col)
idx = find(mask(:) == 1);
if isempty(idx), return; end
m = numel(idx);
V = [e.t(idx) yl(1)*ones(m,1); e.t1(idx) yl(1)*ones(m,1); e.t1(idx) yl(2)*ones(m,1); e.t(idx) yl(2)*ones(m,1)];
F = [(1:m)' (m+1:2*m)' (2*m+1:3*m)' (3*m+1:4*m)'];
patch(a, 'Faces', F, 'Vertices', V, 'FaceColor', col, 'EdgeColor', 'none', 'HandleVisibility', 'off');
end

function segs(a, e, idx, yv, col, lw)
if isempty(idx), return; end
X = [e.t(idx) e.t1(idx) NaN(numel(idx), 1)]'; Y = [yv * ones(numel(idx), 2) NaN(numel(idx), 1)]';
plot(a, X(:), Y(:), '-', 'Color', col, 'LineWidth', lw);
end

function [x, y] = steps(e, v)
% step line with the real epoch durations; NaN leaves a gap
n = numel(v);
x = reshape([e.t(:)'; e.t1(:)'; NaN(1, n)], [], 1);
y = reshape([v(:)'; v(:)'; NaN(1, n)], [], 1);
for k = 1:n-1                                    % join consecutive steps (vertical edge)
    if isfinite(v(k)) && isfinite(v(k+1)), x(3*k) = e.t1(k); y(3*k) = v(k+1); end
end
end

function m = mid(e)
m = 0.5 * (e.t + e.t1);
end

function lab(a, showY, s)
if showY, ylabel(a, s, 'FontName', 'Times New Roman', 'FontSize', 8); end
end

function ev = sessionEvents(D, c, t)
ev = struct('t', zeros(0, 1), 'name', {{}});
mn = {'Solo','Assistive','Collab.'};
for k = 2:size(D, 1)
    add = {};
    if D(k, c.requests) > D(k-1, c.requests), add{end+1} = 'Tool request'; end %#ok<AGROW>
    if D(k, c.deliveries) > D(k-1, c.deliveries), add{end+1} = 'Tool taken'; end %#ok<AGROW>
    if D(k, c.button) == 2 && ~(D(k, c.dialog) == 3 && D(k-1, c.dialog) ~= 3), add{end+1} = 'Pause'; end %#ok<AGROW>
    if any(D(k, c.button) == [1 3]), add{end+1} = 'Resume'; end %#ok<AGROW>
    if D(k, c.dialog) == 1 && D(k-1, c.dialog) ~= 1, add{end+1} = 'Rest warning'; end %#ok<AGROW>
    if D(k, c.dialog) == 3 && D(k-1, c.dialog) ~= 3, add{end+1} = 'Rest'; end %#ok<AGROW>
    if D(k, c.mode) ~= D(k-1, c.mode), add{end+1} = ['\rightarrow ' mn{D(k, c.mode) + 1}]; end %#ok<AGROW>
    for j = 1:numel(add)
        ev.t(end+1, 1) = t(k); ev.name{end+1} = add{j};
    end
end
end

function T = summaryTable(E, titles)
n = numel(E); v = cell(n, 14);
cn = causeNames();
for k = 1:n
    e = E{k}; w = e.t1 - e.t; dur = sum(w);
    share = @(m) 100 * sum(w(m(:) == 1)) / dur;
    stops = numel(find(diff([0; e.aFraw(:) == 10]) == 1));
    looks = numel(find(diff([0; e.away(:) >= 2]) == 1));
    tc = cellfun(@(j) sum(w(e.cause == j)), num2cell(1:numel(cn)));
    [~, ci] = sort(tc, 'descend'); ci = ci(tc(ci) > 0);
    v(k, :) = {titles{k}, dur, share(e.motion == 2), share(e.motion == 1), share(e.motion == 0), share(e.aFraw == 10), ...
        mean(e.cmd(e.cmd > 0), 'omitnan'), min(e.dist, [], 'omitnan'), median(e.dist, 'omitnan'), stops, looks, ...
        max(e.level), max(e.ms, [], 'omitnan'), strjoin(arrayfun(@(j) sprintf('%s %.0f s', cn{j}, tc(j)), ci, 'UniformOutput', false), '; ')};
end
T = cell2table(v, 'VariableNames', {'scenario','duration_s','normal_pct','slow_pct','stopped_pct','protective_stop_pct', ...
    'mean_cmd_speed_dps','min_dist_m','median_dist_m','protective_stops','lookaways_ge_2s','max_fatigue_level', ...
    'max_mental_score','speed_reduced_by'});
end

function used = actionRows(aF, aN)
order = [10 7 4 11 0 9 8 3 6 2 5 1];
v = unique([aF(:); aN(:)]); v = v(isfinite(v));
used = order(ismember(order, v));
end

function L = actionNames()
L = {'Task-layer move','Solo, normal','Solo, slow','Approach','Assistive hold','Collab., normal', ...
     'Collab., slow','Collab., hold','Assist / realloc.','Retreat','Protective stop','Wait (safe position)'};
end

function n = causeNames()
n = {'Fatigue level','Fatigue + distance','Distance','Attention','Low confidence','Sensing','Planner (belief)','Task / worker'};
end

function c = causeColours()
c = [0.90 0.60 0.00; 0.60 0.35 0.10; 0.80 0.15 0.15; 0.55 0.30 0.70; 0.35 0.50 0.70; 0.55 0.55 0.55; 0.00 0.55 0.55; 0.70 0.70 0.68];
end

function c = motionColours()
c = [204 37 41; 230 159 0; 86 180 233; 0 114 178] / 255;    % protective stop, stopped, slow, normal
end

function C = palette()
C = struct('blue',[0 114 178]/255,'verm',[213 94 0]/255,'green',[0 158 115]/255,'orange',[230 159 0]/255, ...
    'red',[204 37 41]/255,'gray',[0.55 0.55 0.55],'light',[0.90 0.90 0.90],'medBg',[1.00 0.95 0.80], ...
    'highBg',[0.99 0.85 0.85],'stopBg',[0.98 0.86 0.86]);
end

function v = ifelse(c, a, b)
if c, v = a; else, v = b; end
end

function v = colOr(D, c, name, d)
if isfield(c, name), v = D(:, c.(name)); else, v = d; end
end

function v = getOpt(s, f, d)
if isfield(s, f) && ~isempty(s.(f)), v = s.(f); else, v = d; end
end

function exportAll(f, base)
exportgraphics(f, [base '.pdf'], 'ContentType', 'vector');
exportgraphics(f, [base '.png'], 'Resolution', 600);
exportgraphics(f, [base '.tif'], 'Resolution', 600);
savefig(f, [base '.fig']);
close(f);
end
