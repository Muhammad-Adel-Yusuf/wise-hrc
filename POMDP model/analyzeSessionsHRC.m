function T = analyzeSessionsHRC(files, opts)
% RESULTS OF ALL RECORDED SESSIONS (journal tables and figures, IEEE style).
%
% T = analyzeSessionsHRC()               every results\scenario_*\scenario_*.mat
% T = analyzeSessionsHRC(files)          cell array of scenario .mat files
% T = analyzeSessionsHRC(files, opts)    opts.out (output folder, default results\analysis_<date>),
%                                        opts.width (3.5 = IEEE single column, 7.16 = double; default 7.16)
%
% Writes into opts.out:
%   sessions.csv          one row per session: participant, condition, duration, share of time at normal /
%                         slow / stopped / protective stop, mean commanded and measured joint speed, min and
%                         median hand-tool distance, protective stops (per minute), share of time each cause
%                         reduced the speed (speedCauseHRC)
%   table_conditions.tex  mean (std) per condition (paste into the paper)
%   figA_speed_vs_state   commanded speed by fatigue level, attention and hand-tool distance
%                         (adaptive sessions, robot working), Kruskal-Wallis tests
%   figB_event_response   commanded speed aligned to each trigger (look-away >= 2 s, fatigue level up,
%                         confidence < 0.5, hand closer than 0.30 m): mean +- SD, reaction latency
%   figC_conditions       adaptive vs fixed baseline (when both were recorded): paired Wilcoxon signed-rank
%                         on per-participant means (rank-sum when not paired)
% Each figure as .pdf (vector), .png (600 dpi) and .fig.
here = fileparts(mfilename('fullpath'));
if nargin < 1 || isempty(files)
    d = dir(fullfile(here, 'results', 'scenario_*', 'scenario_*.mat'));
    files = fullfile({d.folder}, {d.name});
end
if nargin < 2, opts = struct(); end
out = getOpt(opts, 'out', fullfile(here, 'results', ['analysis_' char(datetime('now','Format','yyyyMMdd_HHmmss'))]));
W = getOpt(opts, 'width', 7.16);
if ~exist(out, 'dir'), mkdir(out); end
spd = robotSpeedsHRC();
fs = 8; fn = 'Times New Roman';
c = struct('blue',[0 114 178]/255,'verm',[213 94 0]/255,'green',[0 158 115]/255,'sky',[86 180 233]/255, ...
    'orange',[230 159 0]/255,'red',[204 37 41]/255,'gray',[0.55 0.55 0.55],'purple',[0.55 0.30 0.70]);
causeNames = {'Fatigue level','Fatigue + distance','Distance','Attention','Low confidence','Sensing', ...
    'Planner (belief)','Task / worker'};
% ======================================================= load and derive per-epoch signals
S = {}; rows = {};
for i = 1:numel(files)
    try
        L = load(files{i}, 'rec', 'prm');
    catch err
        warning('analyzeSessionsHRC:load', '%s: %s', files{i}, err.message); continue
    end
    e = epochs(L.rec, L.prm, spd);
    if isempty(e) || numel(e.t) < 10, continue; end
    S{end+1} = e; %#ok<AGROW>
    rows(end+1, :) = sessionRow(e, causeNames); %#ok<AGROW>
end
if isempty(S), error('analyzeSessionsHRC: no usable sessions'); end
vn = [{'session','participant','condition','modes','duration_s','normal_pct','slow_pct','stopped_pct', ...
    'protective_pct','cmd_speed_mean_dps','meas_speed_mean_dps','dist_min_m','dist_median_m','prot_stops', ...
    'prot_stops_per_min'}, matlab.lang.makeValidName(strcat('cause_', causeNames, '_pct'))];
T = cell2table(rows, 'VariableNames', vn);
writetable(T, fullfile(out, 'sessions.csv'));
writeTex(T, fullfile(out, 'table_conditions.tex'));
fprintf('%d sessions -> %s\n', height(T), out);
% ======================================================= figure A: speed vs human state
adapt = cellfun(@(e) strcmp(e.condition, 'adaptive') && e.hasCmd, S);
fprintf('%d of %d sessions recorded the commanded speed (used for figures A and B)\n', sum(adapt), numel(S));
P = pool(S(adapt));
if ~isempty(P.cmd)
    f = newFig(W, 2.3);
    lab = {{'Low','Medium','High'}, {'Focused','Uncertain','Distracted'}, {'<0.15','0.15-0.25','0.25-0.35','>0.35'}};
    grp = {P.level, P.att, discretize(P.dist, [0 0.15 0.25 0.35 Inf]) - 1};
    ttl = {'(a) Fatigue level','(b) Attention','(c) Hand-tool distance (m)'};
    for k = 1:3
        ax = subplot(1, 3, k, 'Parent', f); hold(ax, 'on');
        gk = grp{k}; ok = isfinite(gk) & isfinite(P.cmd);
        if any(ok)
            boxchart(ax, categorical(gk(ok), 0:numel(lab{k})-1, lab{k}), P.cmd(ok), 'BoxFaceColor', c.blue, ...
                'MarkerStyle', '.', 'MarkerColor', c.gray);
            if numel(unique(gk(ok))) > 1
                pk = kruskalwallis(P.cmd(ok), gk(ok), 'off');
                text(ax, 0.98, 0.97, pText(pk), 'Units','normalized','HorizontalAlignment','right', ...
                    'VerticalAlignment','top','FontName',fn,'FontSize',fs-1);
            end
        end
        ylim(ax, [0 1.15*max([spd 5])]);
        title(ax, ttl{k}, 'FontWeight','normal');
        if k == 1, ylabel(ax, 'Commanded speed (deg/s)'); end
        style(ax, fn, fs);
    end
    exportFig(f, fullfile(out, 'figA_speed_vs_state'));
end
% ======================================================= figure B: event-locked response
trig = {'Look-away \geq 2 s','Fatigue level increase','Confidence < 0.5','Hand closer than 0.30 m'};
win = -5:0.5:15;
f = newFig(W, 3.6);
lat = cell(1, 4);
for k = 1:4
    ax = subplot(2, 2, k, 'Parent', f); hold(ax, 'on');
    seg = [];
    for i = find(adapt)
        e = S{i};
        on = triggers(e, k);
        for j = on(:)'
            idx = j + round(win / e.dt);
            if idx(1) < 1 || idx(end) > numel(e.t), continue; end
            s = e.cmd(idx); seg(end+1, :) = s(:)'; %#ok<AGROW>
            pre = e.cmd(j - 1); post = find(e.t(j:end) - e.t(j) <= 15 & e.cmd(j:end) < pre - 1, 1);
            if ~isempty(post), lat{k}(end+1) = e.t(j + post - 1) - e.t(j); end
        end
    end
    if ~isempty(seg)
        m = mean(seg, 1, 'omitnan'); sd = std(seg, 0, 1, 'omitnan');
        fill(ax, [win fliplr(win)], [m - sd fliplr(m + sd)], c.sky, 'EdgeColor','none', 'FaceAlpha', 0.35);
        plot(ax, win, m, '-', 'Color', c.blue, 'LineWidth', 1.2);
    end
    xline(ax, 0, '--', 'Color', c.red, 'LineWidth', 0.8);
    txt = sprintf('n = %d', size(seg, 1));
    if ~isempty(lat{k}), txt = sprintf('%s, latency %.1f s (median)', txt, median(lat{k})); end
    text(ax, 0.98, 0.95, txt, 'Units','normalized','HorizontalAlignment','right','VerticalAlignment','top', ...
        'FontName',fn,'FontSize',fs-1);
    title(ax, sprintf('(%c) %s', 'a' + k - 1, trig{k}), 'FontWeight','normal');
    ylim(ax, [0 1.15*max([spd 5])]); xlim(ax, win([1 end]));
    if mod(k, 2) == 1, ylabel(ax, 'Commanded speed (deg/s)'); end
    if k > 2, xlabel(ax, 'Time from trigger (s)'); end
    style(ax, fn, fs);
end
exportFig(f, fullfile(out, 'figB_event_response'));
% ======================================================= figure C: adaptive vs fixed baseline
conds = unique(T.condition);
if all(ismember({'adaptive','fixed'}, conds))
    kp = {'cmd_speed_mean_dps','slow_pct','prot_stops_per_min','dist_min_m'};
    kn = {'Mean commanded speed (deg/s)','Time at slow speed (%)','Protective stops (1/min)','Min. hand-tool distance (m)'};
    f = newFig(W, 2.2);
    for k = 1:4
        ax = subplot(1, 4, k, 'Parent', f); hold(ax, 'on');
        G = groupsummary(T(ismember(T.condition, {'adaptive','fixed'}), :), {'participant','condition'}, 'mean', kp{k});
        y = G.(['mean_' kp{k}]);
        boxchart(ax, categorical(G.condition, {'fixed','adaptive'}, {'Fixed','Adaptive'}), y, 'BoxFaceColor', c.orange);
        pa = G(strcmp(G.condition,'adaptive'), :); pf = G(strcmp(G.condition,'fixed'), :);
        [both, ia, ib] = intersect(pa.participant, pf.participant);
        if numel(both) >= 2
            p = signrank(pa.(['mean_' kp{k}])(ia), pf.(['mean_' kp{k}])(ib)); tst = 'signed-rank';
        elseif height(pa) >= 1 && height(pf) >= 1
            p = ranksum(pa.(['mean_' kp{k}]), pf.(['mean_' kp{k}])); tst = 'rank-sum';
        else
            p = NaN; tst = '';
        end
        if isfinite(p)
            text(ax, 0.5, 0.97, sprintf('%s (%s)', pText(p), tst), 'Units','normalized', ...
                'HorizontalAlignment','center','VerticalAlignment','top','FontName',fn,'FontSize',fs-1.5);
        end
        title(ax, sprintf('(%c)', 'a' + k - 1), 'FontWeight','normal'); ylabel(ax, kn{k});
        style(ax, fn, fs);
    end
    exportFig(f, fullfile(out, 'figC_conditions'));
else
    fprintf('figure C needs sessions of both conditions (liveHRC: condition = ''adaptive'' / ''fixed'')\n');
end
for k = 1:4
    if ~isempty(lat{k})
        fprintf('  %-26s %3d events, speed reduced after %.1f s (median), IQR %.1f-%.1f s\n', strrep(trig{k},'\geq','>='), ...
            numel(lat{k}), median(lat{k}), prctile(lat{k}, 25), prctile(lat{k}, 75));
    end
end
end

% =========================================================================
function e = epochs(rec, prm, spd)
% per-epoch signals of one session (works with older recordings: missing signals are derived or NaN)
D = rec.data; col = rec.cols; n = size(D, 1); e = [];
if n < 2, return; end
g = @(name) colOr(D, col, name, NaN(n, 1));
e.name = rec.name; e.dt = prm.dt; e.t = g('t');
e.participant = fieldOr(rec, 'participant', 'P?'); e.condition = fieldOr(rec, 'condition', 'adaptive');
e.mode = g('mode'); e.motion = g('motion'); e.level = g('level'); e.att = g('A'); e.dist = g('dHand');
e.conf = g('conf'); e.away = g('awayS'); e.meas = g('jointSpeed');
fa = 'aF'; if isfield(col,'aFraw'), fa = 'aFraw'; end
fnn = 'aN'; if isfield(col,'aNraw'), fnn = 'aNraw'; end
e.aF = g(fa); e.aN = g(fnn);
e.cmd = g('cmdSpeed');
e.hasCmd = any(isfinite(e.cmd));                  % recordings before 2026-10-09 03:00 have no commanded speed: their
                                                  % speeds (other speed settings then) are NOT reconstructed - speed
                                                  % metrics and figures A / B use only sessions that recorded it
e.phase = repmat({''}, n, 1);
if isfield(rec,'phase') && numel(rec.phase) == n, e.phase = rec.phase(:); end
e.cause = g('cause');
if all(isnan(e.cause)) && isfield(rec,'reason') && numel(rec.reason) == n
    for k = 1:n
        o = struct('supervisor', struct('reason', char(rec.reason{k})), 'adaptReason', '', 'speedReason', '');
        if isfield(rec,'adapt') && numel(rec.adapt) >= k, o.adaptReason = char(rec.adapt{k}); end
        e.cause(k) = speedCauseHRC(e.aF(k), e.aN(k), o, e.phase{k}, e.mode(k), 1, '', e.conf(k));
    end
end
end

function r = sessionRow(e, causeNames)
dt = e.dt; n = numel(e.t); dur = n * dt;
pct = @(m) 100 * mean(m, 'omitnan');
on = find(diff([0; e.aF(:) == 10]) == 1);
modes = strjoin(arrayfun(@(m) modeName(m), unique(e.mode(isfinite(e.mode)))', 'UniformOutput', false), '+');
moving = e.cmd > 0;
cz = zeros(1, numel(causeNames));
for k = 1:numel(causeNames), cz(k) = pct(e.cause == k); end
r = [{e.name, e.participant, e.condition, modes, dur, pct(e.motion == 2), pct(e.motion == 1), pct(e.motion == 0), ...
    pct(e.aF == 10), mean(e.cmd(moving), 'omitnan'), mean(e.meas, 'omitnan'), min(e.dist, [], 'omitnan'), ...
    median(e.dist, 'omitnan'), numel(on), numel(on) / (dur / 60)}, num2cell(cz)];
end

function P = pool(S)
P = struct('cmd', [], 'level', [], 'att', [], 'dist', []);
for i = 1:numel(S)
    e = S{i};
    work = strcmp(e.phase, 'WORK') | cellfun(@isempty, e.phase);    % robot working (or phase not recorded)
    P.cmd = [P.cmd; e.cmd(work)]; P.level = [P.level; e.level(work)];
    P.att = [P.att; e.att(work)]; P.dist = [P.dist; e.dist(work)];
end
end

function on = triggers(e, k)
% epochs at which trigger k starts
switch k
    case 1
        if any(isfinite(e.away)), x = e.away >= 2; else, x = e.att == 2; end
    case 2
        x = [false; diff(e.level) > 0];
        on = find(x); return
    case 3
        x = e.conf < 0.5;
    case 4
        x = e.dist < 0.30;
end
x(isnan(x)) = 0;
on = find(diff([0; x(:)]) == 1);
end

function writeTex(T, file)
% mean (std) of the main metrics per condition, LaTeX table body
m = {'duration_s','normal_pct','slow_pct','protective_pct','cmd_speed_mean_dps','dist_min_m','prot_stops_per_min'};
h = {'Duration (s)','Normal (\%)','Slow (\%)','Prot. stop (\%)','Speed (deg/s)','$d_{\min}$ (m)','Stops/min'};
fid = fopen(file, 'w');
fprintf(fid, '%% generated by analyzeSessionsHRC (%s)\n\\begin{tabular}{l%s}\n\\hline\nCondition & %s \\\\\n\\hline\n', ...
    char(datetime('now')), repmat('c', 1, numel(m)), strjoin(h, ' & '));
for cnd = unique(T.condition)'
    R = T(strcmp(T.condition, cnd{1}), :);
    v = cellfun(@(f) sprintf('%.2f (%.2f)', mean(R.(f), 'omitnan'), std(R.(f), 'omitnan')), m, 'UniformOutput', false);
    fprintf(fid, '%s (n=%d) & %s \\\\\n', cnd{1}, height(R), strjoin(v, ' & '));
end
fprintf(fid, '\\hline\n\\end{tabular}\n');
fclose(fid);
end

function f = newFig(W, H)
f = figure('Color','w','Units','inches','Position',[1 1 W H],'PaperUnits','inches', ...
    'PaperPosition',[0 0 W H],'PaperSize',[W H],'Visible','off');
end

function style(ax, fn, fs)
set(ax,'FontName',fn,'FontSize',fs,'LineWidth',0.5,'Box','on','TickDir','in','XGrid','on','YGrid','on', ...
    'GridLineStyle',':','GridAlpha',0.35,'Layer','top');
set(get(ax,'Title'),'FontName',fn,'FontSize',fs);
end

function exportFig(f, base)
try
    exportgraphics(f, [base '.pdf'], 'ContentType', 'vector');
    exportgraphics(f, [base '.png'], 'Resolution', 600);
catch err
    warning('analyzeSessionsHRC:export', '%s', err.message);
end
savefig(f, [base '.fig']);
close(f);
end

function s = pText(p)
if p < 0.001, s = 'p < 0.001'; else, s = sprintf('p = %.3f', p); end
end

function v = colOr(D, col, name, d)
if isfield(col, name), v = D(:, col.(name)); else, v = d; end
end

function v = fieldOr(s, f, d)
if isfield(s, f) && ~isempty(s.(f)), v = s.(f); else, v = d; end
end

function v = getOpt(s, f, d)
if isfield(s, f) && ~isempty(s.(f)), v = s.(f); else, v = d; end
end

function s = modeName(m)
names = {'solo','assistive','collaborative'};
s = names{m + 1};
end
