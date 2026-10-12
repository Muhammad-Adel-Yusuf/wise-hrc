function fig = sessionFigureHRC(rec, prm, opts)
% Journal figure of ONE recorded scenario of the live test (from START, key G,
% to STOP, key X). Made automatically by liveHRC; it can also be re-made later
% from the saved .mat file:   S = load('scenario_...mat'); sessionFigureHRC(S.rec, S.prm);
%
% Panels (shared time axis, t = 0 at START):
%   (a) task mode (solo / assistive / collaborative)
%   (b) fatigue SENSOR signals of the camera program: physical fatigue index f and
%       mental fatigue score (0..1) with their level thresholds (dotted medium,
%       dashed high); recordings without them: the levels
%   (c) attention SENSOR signal: time looking away from the work (s) with the
%       2 s distraction limit; recordings without it: the attention level
%   (d) distance between the worker's hand and the robot tool, with the
%       prohibited / collaborative / assistive distances; gray background =
%       safety geometry not verified (no distance available)
%   (e) robot joint speed (deg/s): commanded by the decision layer (dashed,
%       piecewise constant) and measured on the robot (solid) - how the robot
%       answers (b)-(d); recordings without them: measured tool speed (m/s)
%   (f) cause of a speed reduction (speedCauseHRC): which signal / rule slowed or
%       stopped the robot (fatigue, distance, attention, estimate confidence, ...)
%   (g) robot action executed (colour = robot motion) and the planner's
%       nominal action (gray line, visible where the safety supervisor or the
%       fatigue adaptation changed it)
% Vertical dotted lines: events (tool request, tool taken, pause, resume,
% rest warning, rest, mode change).
%
% rec  : struct from liveHRC (rec.data: one row per decision epoch, columns in rec.cols)
% prm  : stateDefinitionHRC (distances)
% opts : optional fields  width (inches, 3.5 = IEEE single column, 7.16 = double
%        column; default 3.5), height (default 7.0), file (base name: writes
%        file.pdf (vector), file.png (600 dpi) and file.fig), title ('' = none)
% IEEE style: Times New Roman 8 pt, 0.5 pt axes, colour-blind safe colours
% (Okabe-Ito) with different line styles, so the figure also reads in gray scale.
if nargin < 3, opts = struct(); end
W = getOpt(opts,'width',3.5);
Hgt = getOpt(opts,'height',7.0);
file = getOpt(opts,'file','');
ttl = getOpt(opts,'title','');
fs = 8; fn = 'Times New Roman';
c = struct('blue',[0 114 178]/255,'verm',[213 94 0]/255,'green',[0 158 115]/255, ...
    'sky',[86 180 233]/255,'orange',[230 159 0]/255,'red',[204 37 41]/255, ...
    'gray',[0.55 0.55 0.55],'light',[0.90 0.90 0.90]);
D = rec.data; col = rec.cols;
g = @(name) D(:, col.(name));
t = g('t'); n = numel(t);
dt = prm.dt;
if n < 2
    warning('sessionFigureHRC: fewer than 2 epochs recorded - no figure');
    fig = [];
    return
end
tEnd = t(end) + dt;
fig = figure('Name',['Scenario: ' rec.name],'NumberTitle','off','Color','w', ...
    'Units','inches','Position',[1 1 W Hgt],'PaperUnits','inches', ...
    'PaperPosition',[0 0 W Hgt],'PaperSize',[W Hgt]);
% ---------------- panel layout (inches): left margin for the action names
labs = actionNames();
aF = g('aF'); aN = g('aN');
used = actionRows(aF, aN);
left = 1.10; right = 0.10; top = 0.45 + 0.20*~isempty(ttl); bottom = 0.68; gap = 0.08;
NP = 7;                                           % (a) mode (b) fatigue (c) attention (d) distance (e) speed (f) cause (g) action
hasCause = isfield(col,'cause');
if ~hasCause && isfield(rec,'reason') && numel(rec.reason) == n   % older recordings: cause derived from the logged reasons
    D(:, end+1) = deriveCause(rec, D, col, n); col.cause = size(D, 2); hasCause = true;
end
causeRows = [];
if hasCause, causeRows = unique(D(isfinite(D(:,col.cause)) & D(:,col.cause) > 0, col.cause))'; end
wts = [0.42 0.85 0.75 1.00 0.85 0.25 + 0.16*max(1, numel(causeRows)) 0.30 + 0.16*numel(used)];
hAvail = Hgt - top - bottom - gap*(NP-1);
hs = hAvail * wts/sum(wts);
ax = gobjects(1,NP);
y = Hgt - top;
for i = 1:NP
    y = y - hs(i);
    ax(i) = axes(fig,'Units','inches','Position',[left y W-left-right hs(i)]); %#ok<LAXES>
    y = y - gap;
    hold(ax(i),'on');
end
% ---------------- events
ev = sessionEvents(rec);
for i = 1:NP
    for k = 1:numel(ev.t)
        xline(ax(i), ev.t(k), ':', 'Color', c.gray, 'LineWidth', 0.6, 'HandleVisibility','off');
    end
end
row = zeros(size(ev.t));                          % event names above panel (a), staggered when close
for k = 2:numel(ev.t)
    if ev.t(k) - ev.t(k-1) < 0.12*tEnd, row(k) = mod(row(k-1) + 1, 2); end
end
for k = 1:numel(ev.t)
    text(ax(1), ev.t(k), 2.55 + 0.75*row(k), ev.name{k}, 'FontName',fn,'FontSize',fs-1.5, ...
        'HorizontalAlignment','center','VerticalAlignment','bottom', ...
        'Color',[0.25 0.25 0.25],'Clipping','off');
end
% ---------------- (a) task mode
a = ax(1);
[xm,ym] = stairsXY(t, g('mode'), dt);
plot(a, xm, ym, '-', 'Color', 'k', 'LineWidth', 1.2);
set(a,'YLim',[-0.45 2.45],'YTick',0:2,'YTickLabel',{'Solo','Assistive','Collab.'});
ylabel(a,'Task mode');
% ---------------- (b) fatigue
a = ax(2);
if hasCol(col,'physicalF') && any(isfinite(g('physicalF'))) || hasCol(col,'mentalScore') && any(isfinite(g('mentalScore')))
    hFp = plot(a, t + dt/2, g('physicalF'), '-', 'Color', c.blue, 'LineWidth', 1.1);
    hFm = plot(a, t + dt/2, g('mentalScore'), '--', 'Color', c.verm, 'LineWidth', 1.1);
    plot(a, [0 tEnd], [0.3 0.3], ':', 'Color', c.blue, 'LineWidth', 0.7);       % physical medium / high
    plot(a, [0 tEnd], [0.6 0.6], '-.', 'Color', c.blue, 'LineWidth', 0.7);
    plot(a, [0 tEnd], [0.4 0.4], ':', 'Color', c.verm, 'LineWidth', 0.7);       % mental medium / high
    plot(a, [0 tEnd], [0.7 0.7], '-.', 'Color', c.verm, 'LineWidth', 0.7);
    hQ = plot(a, t + dt/2, g('conf'), '-', 'Color', c.gray, 'LineWidth', 0.8);   % confidence of the estimate
    set(a,'YLim',[0 1.45],'YTick',0:0.25:1);                                     % top band: legend
    ylabel(a,'Fatigue (0-1)');
    lg = legend(a,[hFp hFm hQ],{'Physical index f','Mental score','Confidence'},'Orientation','horizontal', ...
        'Location','north','Box','off');
else
    [x1,y1] = stairsXY(t, g('Fp'), dt);
    [x2,y2] = stairsXY(t, g('Fm'), dt);
    hFp = plot(a, x1, y1 + 0.05, '-', 'Color', c.blue, 'LineWidth', 1.2);
    hFm = plot(a, x2, y2 - 0.05, '--', 'Color', c.verm, 'LineWidth', 1.2);
    set(a,'YLim',[-0.35 3.0],'YTick',0:2,'YTickLabel',{'Low','Medium','High'});  % top band: legend
    ylabel(a,'Fatigue');
    lg = legend(a,[hFp hFm],{'Physical (F^p)','Mental (F^m)'},'Orientation','horizontal', ...
        'Location','north','Box','off');
end
lg.ItemTokenSize = [14 8];
% ---------------- (c) attention
a = ax(3);
if hasCol(col,'awayS') && any(isfinite(g('awayS')))
    aw = g('awayS');
    aTop = 1.45*max(prm.distractionLimit + 1, max(aw,[],'omitnan'));
    hA = plot(a, t + dt/2, aw, '-', 'Color', c.green, 'LineWidth', 1.1);
    hL = plot(a, [0 tEnd], prm.distractionLimit*[1 1], '--', 'Color', c.red, 'LineWidth', 0.8);
    set(a,'YLim',[0 aTop]);
    ylabel(a,'Away (s)');
    lg = legend(a,[hA hL],{'Time looking away',sprintf('Limit %.0f s',prm.distractionLimit)}, ...
        'Orientation','horizontal','Location','north','Box','off');
    lg.ItemTokenSize = [14 8];
else
    [x3,y3] = stairsXY(t, g('A'), dt);
    plot(a, x3, y3, '-', 'Color', c.green, 'LineWidth', 1.2);
    set(a,'YLim',[-0.35 2.35],'YTick',0:2,'YTickLabel',{'Focused','Uncertain','Distracted'});
    ylabel(a,'Attention');
end
% ---------------- (d) hand-robot distance
a = ax(4);
fixedA = isfield(prm,'assistZone') && prm.assistZone.enabled;   % assistive zone = circle at the handover point
nth = 3 - fixedA;
th = [prm.dProhibit prm.dCollabMax prm.dAssistMax];
yTop = 1.55*max([1.0, th(nth) + 0.10, max(g('dHand'),[],'omitnan') + 0.05]);  % top band: legend
unv = g('verified')==0;
shadeMask(a, t, unv, dt, [0 yTop], c.light);
hD = plot(a, t + dt/2, g('dHand'), '-', 'Color', 'k', 'LineWidth', 1.1);
ls = {'-','-.',':'}; tc = {c.red, c.orange, c.blue};
hT = gobjects(1,nth);
for i = 1:nth
    hT(i) = plot(a, [0 tEnd], th(i)*[1 1], ls{i}, 'Color', tc{i}, 'LineWidth', 0.9);
end
set(a,'YLim',[0 yTop]);
ylabel(a,'Distance (m)');
hU = patch(a, NaN, NaN, c.light, 'EdgeColor','none');
names = {'Hand-tool', sprintf('Prohib. %.2f',th(1)), sprintf('Collab. %.2f',th(2)), ...
    sprintf('Assist. %.2f',th(3))};
names = names(1:nth+1);
hh = [hD hT];
if any(unv), hh = [hh hU]; names{end+1} = 'Not verified'; end
lg = legend(a, hh, names, 'Location','north','Box','off','NumColumns',3);
lg.ItemTokenSize = [14 8];
% ---------------- (e) robot speed (measured tool speed)
a = ax(5);
if hasCol(col,'cmdSpeed') && any(isfinite(g('cmdSpeed')))
    vc = g('cmdSpeed'); vm = g('jointSpeed');
    vTop = 1.45*max([5; vc; vm],[],'omitnan');
    shadeMask(a, t, aF==10, dt, [0 vTop], [0.98 0.86 0.86]);                   % protective stop
    [xc,yc] = stairsXY(t, vc, dt);
    hC = plot(a, xc, yc, '--', 'Color', 'k', 'LineWidth', 1.0);
    hM = plot(a, t + dt/2, vm, '-', 'Color', c.blue, 'LineWidth', 1.0);
    set(a,'YLim',[0 vTop]);
    ylabel(a,'Speed (deg/s)');
    hh = [hC hM]; nm = {'Commanded','Measured'};
else
    v = g('speed');
    vTop = max(0.05, 1.25*max(v,[],'omitnan'));
    shadeMask(a, t, aF==10, dt, [0 vTop], [0.98 0.86 0.86]);                   % protective stop
    hM = plot(a, t + dt/2, v, '-', 'Color', c.blue, 'LineWidth', 1.1);
    set(a,'YLim',[0 vTop]);
    ylabel(a,'Speed (m/s)');
    hh = hM; nm = {'Measured tool speed'};
end
if any(aF==10)
    hh(end+1) = patch(a, NaN, NaN, [0.98 0.86 0.86], 'EdgeColor','none'); nm{end+1} = 'Protective stop';
end
lg = legend(a, hh, nm, 'Orientation','horizontal','Location','north','Box','off');
lg.ItemTokenSize = [14 8];
% ---------------- (f) cause of a speed reduction
a = ax(6);
cnames = {'Fatigue level','Fatigue + distance','Distance','Attention','Low confidence','Sensing','Planner (belief)','Task / worker'};
ccol = [0.90 0.60 0.00; 0.60 0.35 0.10; 0.80 0.15 0.15; 0.55 0.30 0.70; 0.35 0.50 0.70; 0.55 0.55 0.55; 0.00 0.55 0.55; 0.70 0.70 0.68];
if hasCause && ~isempty(causeRows)
    cz = D(:, col.cause);
    for k = 1:numel(causeRows)
        idx = find(cz == causeRows(k));
        X = [t(idx) t(idx)+dt NaN(numel(idx),1)]'; Y = [k*ones(numel(idx),1) k*ones(numel(idx),1) NaN(numel(idx),1)]';
        plot(a, X(:), Y(:), '-', 'Color', ccol(causeRows(k),:), 'LineWidth', 4);
    end
    set(a,'YLim',[0.4 numel(causeRows)+0.6],'YTick',1:numel(causeRows),'YTickLabel',cnames(causeRows));
else
    set(a,'YLim',[0 1],'YTick',0.5,'YTickLabel',{'None'});
end
ylabel(a,'Reduced by');
% ---------------- (g) robot actions
a = ax(7);
rowOf = @(v) rowIndex(v, used);
mot = g('motion');
motCol = {c.red, c.orange, c.sky, c.blue};      % protective stop, stopped, slow, normal
motLvl = [-1 0 1 2];
yN = rowOf(aN);
[xn,yn] = stairsXY(t, yN, dt);
hN = plot(a, xn, yn, '-', 'Color', c.gray, 'LineWidth', 0.6);
hM = gobjects(1,4);
for m = 1:4
    idx = find(mot==motLvl(m));
    yF = rowOf(aF(idx));
    X = [t(idx) t(idx)+dt NaN(numel(idx),1)]'; Y = [yF yF NaN(numel(idx),1)]';
    if isempty(idx), X = NaN; Y = NaN; end           % this motion never occurs: empty legend handle
    hM(m) = plot(a, X(:), Y(:), '-', 'Color', motCol{m}, 'LineWidth', 4);
end
set(a,'YLim',[0.4 numel(used)+0.6],'YTick',1:numel(used),'YTickLabel',labs(used + 1));
ylabel(a,'Robot action');
xlabel(a,'Time (s)');
present = arrayfun(@(m) any(mot==motLvl(m)), 1:4);
mn = {'Protective stop','Stopped','Slow','Normal speed'};
lg = legend(a, [hM(present) hN], [mn(present) {'Planner (nominal)'}], ...
    'Orientation','horizontal','Box','off','NumColumns',3);
lg.ItemTokenSize = [14 8];
lgD = lg;                                         % placed under the time axis after the fonts are set
% ---------------- common style
pl = {'(a)','(b)','(c)','(d)','(e)','(f)','(g)'};
axW = W - left - right;
for i = 1:NP
    set(get(ax(i),'YLabel'),'Units','normalized','Position',[-(left - 0.14)/axW 0.5 0], ...
        'FontName',fn,'FontSize',fs);
    set(ax(i),'XLim',[0 tEnd],'FontName',fn,'FontSize',fs,'LineWidth',0.5,'Box','on', ...
        'TickDir','in','TickLength',[0.01 0.01],'XGrid','on','YGrid','on', ...
        'GridLineStyle',':','GridAlpha',0.35,'Layer','top');
    if i < NP, set(ax(i),'XTickLabel',[]); end
    text(ax(i), 0.008, 0.97, pl{i}, 'Units','normalized','FontName',fn,'FontSize',fs, ...
        'FontWeight','bold','VerticalAlignment','top','HorizontalAlignment','left', ...
        'BackgroundColor','w','Margin',0.5);
end
set(findall(fig,'Type','legend'),'FontName',fn,'FontSize',fs-1.5);
for lgx = findall(fig,'Type','legend')', lgx.ItemTokenSize = [10 6]; end   % narrower legends: clear of the panel letters
set(lgD,'Units','inches','Position',[left 0.02 axW 0.26]);
linkaxes(ax,'x');
if ~isempty(ttl)
    annotation(fig,'textbox',[0 1-0.2/Hgt 1 0.2/Hgt],'String',ttl,'EdgeColor','none', ...
        'HorizontalAlignment','center','FontName',fn,'FontSize',fs+1,'FontWeight','bold');
end
% ---------------- export
if ~isempty(file)
    try
        exportgraphics(fig, [file '.pdf'], 'ContentType','vector');
        exportgraphics(fig, [file '.png'], 'Resolution',600);
    catch err
        warning('sessionFigureHRC:export','export failed (%s): saved as .fig only', err.message);
    end
    savefig(fig, [file '.fig']);
end
end

% =========================================================================
function cz = deriveCause(rec, D, col, n)
% speedCauseHRC applied to the logged supervisor / adaptation reasons of each epoch
fa = 'aF'; if isfield(col,'aFraw'), fa = 'aFraw'; end
fn = 'aN'; if isfield(col,'aNraw'), fn = 'aNraw'; end
cz = zeros(n, 1);
for k = 1:n
    out = struct('supervisor', struct('reason', char(rec.reason{k})), 'adaptReason', '', 'speedReason', '');
    if isfield(rec,'adapt') && numel(rec.adapt) >= k, out.adaptReason = char(rec.adapt{k}); end
    ph = ''; if isfield(rec,'phase') && numel(rec.phase) >= k, ph = char(rec.phase{k}); end
    cq = NaN; if isfield(col,'conf'), cq = D(k, col.conf); end
    cz(k) = speedCauseHRC(D(k, col.(fa)), D(k, col.(fn)), out, ph, D(k, col.mode), 1, '', cq);
end
end

function tf = hasCol(col, name)
tf = isfield(col, name);
end

function v = getOpt(s, f, d)
if isfield(s, f) && ~isempty(s.(f)), v = s.(f); else, v = d; end
end

function r = rowIndex(v, used)
[~, r] = ismember(v, used);
r = double(r);
r(r==0) = NaN;
end

function L = actionNames()
% index = action + 1 (0 = task-layer transfer)
L = {'Task-layer move','Solo, normal','Solo, slow','Approach','Assistive hold', ...
    'Collab., normal','Collab., slow','Collab., hold','Assist / realloc.','Retreat','Protective stop', ...
    'Wait (safe position)'};                      % 11: retreat while already at the safe position
end

function used = actionRows(aF, aN)
% actions that occur, bottom (protective stop) to top (full speed)
order = [10 7 4 11 0 9 8 3 6 2 5 1];
v = unique([aF(:); aN(:)]);
v = v(~isnan(v));
used = order(ismember(order, v));
if isempty(used), used = 10; end
end

function [x, y] = stairsXY(t, v, dt)
% step line, one level per epoch; NaN (not measured) leaves a gap
n = numel(t);
x = reshape([t(:)'; t(:)' + dt; NaN(1,n)], [], 1);
y = reshape([v(:)'; v(:)'; NaN(1,n)], [], 1);
x = x(1:end-1); y = y(1:end-1);
for k = 1:n-1                                    % join consecutive equal-time steps
    if ~isnan(v(k)) && ~isnan(v(k+1))
        y(3*k) = v(k+1); x(3*k) = t(k) + dt;
    end
end
end

function shadeMask(a, t, mask, dt, yl, col)
idx = find(mask);
if isempty(idx), return; end
m = numel(idx);
V = [t(idx) yl(1)*ones(m,1); t(idx)+dt yl(1)*ones(m,1); t(idx)+dt yl(2)*ones(m,1); t(idx) yl(2)*ones(m,1)];
F = [(1:m)' (m+1:2*m)' (2*m+1:3*m)' (3*m+1:4*m)'];
patch(a,'Faces',F,'Vertices',V,'FaceColor',col,'EdgeColor','none','HandleVisibility','off');
end

function ev = sessionEvents(rec)
% events from the recorded epochs
D = rec.data; col = rec.cols; t = D(:,col.t);
ev = struct('t',zeros(0,1),'name',{{}});
modeName = {'Solo','Assistive','Collab.'};
for k = 2:size(D,1)
    add = {};
    if D(k,col.requests) > D(k-1,col.requests), add{end+1} = 'Tool request'; end %#ok<AGROW>
    if D(k,col.deliveries) > D(k-1,col.deliveries), add{end+1} = 'Tool taken'; end %#ok<AGROW>
    if D(k,col.button)==2, add{end+1} = 'Pause'; end %#ok<AGROW>
    if any(D(k,col.button)==[1 3]), add{end+1} = 'Resume'; end %#ok<AGROW>
    if D(k,col.dialog)==1 && D(k-1,col.dialog)~=1, add{end+1} = 'Rest warning'; end %#ok<AGROW>
    if D(k,col.dialog)==3 && D(k-1,col.dialog)~=3, add{end+1} = 'Rest'; end %#ok<AGROW>
    if D(k,col.mode)~=D(k-1,col.mode), add{end+1} = ['\rightarrow ' modeName{D(k,col.mode)+1}]; end %#ok<AGROW>
    for j = 1:numel(add)
        ev.t(end+1,1) = t(k);
        ev.name{end+1} = add{j};
    end
end
end
