function st = plotStyleHRC()
% Common figure style of the HRC test figures (plotSoloHRC, plotToolFetchHRC,
% testSoloCasesHRC, sweepToolFetchHRC). Every figure uses the SAME colors
% for the same quantity, so figures can be compared side by side:
%
%   human state     physical fatigue Fp = blue, mental fatigue Fm = orange,
%                   attention A = green; true value solid, robot belief dashed
%   distances       hand-to-tool = blue, upper-limb clearance = orange,
%                   thresholds = gray dashed lines labelled on the right
%   zone / risk     green = safe, yellow = caution, orange = close,
%                   red = danger, gray = unknown
%   robot action    shade of the action = commanded robot motion:
%                   dark blue normal speed, light blue slow, yellow stopped
%                   (hold / wait), red protective stop; gray line = what the
%                   planner suggested (visible only where the supervisor changed it)
%   protective stop light red background in every panel
%
% st.<color> are RGB rows. Helper functions (handles):
%   ax = st.stack(fig, weights, ttl, subttl)   stacked panels sharing the time axis
%   st.finish(ax, t)                           common axis limits / ticks / fonts
%   st.legend(ax, h, labels)                   legend at the right of a panel
%   st.threshold(ax, t, y, txt)                labelled dashed threshold line
%   st.strip(ax, t, rows, colors, names)       colored state strips (zone, risk, phase)
%   st.actions(ax, t, aN, aF [, motion])       action timeline
%   st.shade(ax, t, mask, color)               background shading of the steps in mask
%   st.caseBars(fig, names, aF, valueTxt, ttl, subttl)
%                                              one bar per case: share of the time spent
%                                              at normal speed / slow / stopped / protective stop
%   st.save(fig, fileName)                     save a 200 dpi PNG
hex = @(h) sscanf(h(2:end),'%2x%2x%2x',[1 3])/255;
st.blue    = hex('#2a78d6');
st.orange  = hex('#eb6834');
st.green   = hex('#1baf7a');
st.violet  = hex('#4a3aa7');
st.ink     = hex('#0b0b0b');
st.ink2    = hex('#52514e');
st.muted   = hex('#898781');
st.grid    = hex('#e1e0d9');
st.good    = hex('#0ca30c');
st.warning = hex('#fab219');
st.serious = hex('#ec835a');
st.critical= hex('#d03b3b');
st.unknown = hex('#c3c2b7');
st.stopShade = hex('#f7d9d9');
st.normalMotion = hex('#1c5cab');
st.slowMotion   = hex('#86b6ef');
st.stopped      = hex('#fab219');
st.protective   = hex('#d03b3b');
st.font = 'Helvetica';
st.fontSize = 9;
st.titleSize = 12;
st.lw = 1.8;               % data lines
st.lwThin = 1.1;           % belief / reference lines
% zone and risk colors (index = value + 1); NaN -> st.unknown
st.zoneColors = [st.critical; st.serious; st.warning; st.good];
st.zoneNames  = {'prohibited','collaborative','assistive','separated'};
st.riskColors = [st.good; st.warning; st.critical];
st.riskNames  = {'none','potential','active'};
% actions: order on the y axis (bottom = protective stop, top = full speed)
st.actionOrder = [10 7 4 0 9 8 3 6 2 5 1];
st.actionLabels = {'ps  protective stop','ch  collab. hold','ah  assist hold', ...
    'tr  task layer (move / rest)','rt  retreat','ra  assist / realloc.','ap  approach', ...
    'cs  collab. slow','ss  solo slow','cn  collab. normal','sc  solo continue'};
st.motionOf = @(a) motionLevel(a);
st.stack = @stack;
st.finish = @finish;
st.legend = @rightLegend;
st.threshold = @threshold;
st.strip = @strip;
st.actions = @(ax,t,aN,aF,varargin) actions(ax,t,aN,aF,st,varargin{:});
st.shade = @shade;
st.caseBars = @(fig,names,aF,valueTxt,ttl,subttl) caseBars(fig,names,aF,valueTxt,ttl,subttl,st);
st.save = @saveFig;
end

% =========================================================================
function m = motionLevel(a)
% commanded motion: 0 stopped (hold/wait), 1 slow, 2 normal, -1 protective stop
act = actionDefinitionHRC();
m = zeros(size(a));
for i = 1:numel(a)
    if a(i)==0
        m(i) = 1;                                 % task-layer transfer moves slowly
    elseif a(i)==10
        m(i) = -1;
    else
        m(i) = act.motion(a(i));
    end
end
end

function ax = stack(fig, weights, ttl, subttl)
% stacked panels with a common time axis; legends go to the right
fig = figure(fig); clf
set(fig,'Color','w','Units','pixels');
p = get(fig,'Position');
set(fig,'Position',[p(1) max(p(2)+p(4)-940,40) 1150 940]);
left = 0.20; width = 0.56; top = 0.905; bottom = 0.06; gap = 0.018;
n = numel(weights);
h = (top - bottom - gap*(n-1))*weights/sum(weights);
ax = gobjects(n,1);
y = top;
for i = 1:n
    y = y - h(i);
    ax(i) = axes('Parent',fig,'Position',[left y width h(i)]);
    y = y - gap;
end
annotation(fig,'textbox',[0.02 0.94 0.96 0.05],'String',ttl,'EdgeColor','none', ...
    'FontSize',13,'FontWeight','bold','Interpreter','none','VerticalAlignment','bottom');
if nargin > 3 && ~isempty(subttl)
    annotation(fig,'textbox',[0.02 0.912 0.96 0.03],'String',subttl,'EdgeColor','none', ...
        'FontSize',9,'Color',[0.32 0.32 0.31],'Interpreter','none','VerticalAlignment','bottom');
end
end

function finish(ax, t)
% common look: same time limits, light grid, time labels only on the bottom panel
dt = t(2) - t(1);
for i = 1:numel(ax)
    set(ax(i),'XLim',[t(1) t(end)+dt],'FontName','Helvetica','FontSize',9, ...
        'Box','on','XGrid','on','YGrid','on','GridColor',[0.80 0.80 0.78],'GridAlpha',0.6, ...
        'XColor',[0.32 0.32 0.31],'YColor',[0.32 0.32 0.31],'TickDir','out','Layer','top', ...
        'TickLabelInterpreter','none');
    if i < numel(ax)
        set(ax(i),'XTickLabel',[]);
    end
    ax(i).YLabel.FontSize = 9.5;
    ax(i).YLabel.FontWeight = 'bold';
    ax(i).YLabel.Color = [0.20 0.20 0.20];
end
xlabel(ax(end),'Time [s]','FontWeight','bold');
linkaxes(ax,'x');
end

function rightLegend(ax, h, labels)
% legend placed right of the panel, top aligned (keeps all panels the same width)
lg = legend(ax, h, labels, 'Box','off','FontSize',8.5,'Interpreter','tex', ...
    'AutoUpdate','off');
lg.Units = 'normalized';
p = ax.Position;
lp = lg.Position;
lg.Position = [p(1)+p(3)+0.012, p(2)+p(4)-lp(4), lp(3), lp(4)];
end

function threshold(ax, t, y, txt)
% dashed reference line with its label inside the right end of the panel
dt = t(2) - t(1);
hold(ax,'on')
plot(ax,[t(1) t(end)+dt],[y y],'--','Color',[0.54 0.53 0.51],'LineWidth',0.9, ...
    'HandleVisibility','off');
text(ax, t(end)+dt, y, [txt ' '], 'HorizontalAlignment','right','VerticalAlignment','bottom', ...
    'FontSize',7.5,'Color',[0.45 0.45 0.43]);
end

function strip(ax, t, rows, colors, names)
% rows   : nRow x n values (NaN = unknown), drawn one strip per row, top row first
% colors : cell, colors{r} = (nValues x 3) colors of row r (value v -> row v+1)
% names  : row names shown on the y axis
dt = t(2) - t(1);
nR = size(rows,1);
hold(ax,'on')
for r = 1:nR
    y0 = nR - r + 0.6;                            % row r, top row first
    v = rows(r,:);
    k = 1;
    while k <= numel(v)
        m = 1;
        while k+m <= numel(v) && isequaln(v(k+m), v(k)), m = m+1; end
        if isnan(v(k))
            c = [0.76 0.76 0.72];
        else
            c = colors{r}(v(k)+1,:);
        end
        patch(ax,[t(k) t(k)+m*dt t(k)+m*dt t(k)],[y0 y0 y0+0.8 y0+0.8],c, ...
            'EdgeColor','w','LineWidth',0.5,'HandleVisibility','off');
        k = k + m;
    end
end
set(ax,'YLim',[0.4 nR+0.6],'YTick',1:nR,'YTickLabel',fliplr(names),'YGrid','off');
end

function h = actions(ax, t, aN, aF, st, mot)
% action timeline: each step drawn at the level of its final action and
% colored by the commanded robot motion; gray = planner (nominal) action
% mot (optional): logged motion per step (-1 protective stop, 0 stopped,
% 1 slow, 2 normal), e.g. a task-layer transfer that waits at its target
dt = t(2) - t(1);
lev = @(a) arrayfun(@(x) find(st.actionOrder==x,1), a);
yN = lev(aN(:)'); yF = lev(aF(:)');
hold(ax,'on')
hN = stairs(ax,[t(:)' t(end)+dt],[yN yN(end)],'Color',[0.62 0.62 0.60],'LineWidth',1.0);
if nargin < 6 || isempty(mot)
    mot = st.motionOf(aF(:)');
else
    mot = mot(:)';
end
cols = {st.protective, st.stopped, st.slowMotion, st.normalMotion};
lvl = [-1 0 1 2];
hc = gobjects(1,4);
for c = 1:4
    hc(c) = plot(ax,NaN,NaN,'-','Color',cols{c},'LineWidth',6);
    k = find(mot==lvl(c));
    for j = k
        plot(ax,[t(j) t(j)+dt],[yF(j) yF(j)],'-','Color',cols{c},'LineWidth',6, ...
            'HandleVisibility','off');
    end
end
set(ax,'YLim',[0.3 numel(st.actionOrder)+0.7],'YTick',1:numel(st.actionOrder), ...
    'YTickLabel',st.actionLabels,'FontSize',8);
h = [hc hN];
end

function shade(ax, t, mask, color)
% light background over the steps where mask is true (drawn behind the data)
% call it AFTER the y limits of the panel are final
dt = t(2) - t(1);
mask = mask(:)';
yl = ax.YLim;
hold(ax,'on')
k = 1;
while k <= numel(mask)
    if mask(k)
        m = 1;
        while k+m <= numel(mask) && mask(k+m), m = m+1; end
        p = patch(ax,[t(k) t(k)+m*dt t(k)+m*dt t(k)],[yl(1) yl(1) yl(2) yl(2)],color, ...
            'EdgeColor','none','HandleVisibility','off');
        uistack(p,'bottom');
        k = k + m;
    else
        k = k + 1;
    end
end
ax.YLim = yl;
end

function caseBars(fig, names, aF, valueTxt, ttl, subttl, st)
% names : case names (cell), aF : cell of final-action vectors, valueTxt : text
% written at the right of each bar (e.g. work done or completion time)
nC = numel(names);
share = zeros(nC,4);                              % normal, slow, stopped, protective
for i = 1:nC
    m = st.motionOf(aF{i}(:)');
    share(i,:) = 100*[sum(m==2) sum(m==1) sum(m==0) sum(m==-1)]/numel(m);
end
fig = figure(fig); clf                            % figure number -> figure handle
set(fig,'Color','w','Units','pixels');
p = get(fig,'Position');
hFig = 160 + 26*nC;
set(fig,'Position',[p(1) max(p(2)+p(4)-hFig,40) 1100 hFig]);
ax = axes('Parent',fig,'Units','pixels','Position',[330 70 560 hFig-150]);
b = barh(ax, share, 0.7, 'stacked', 'EdgeColor','w','LineWidth',0.8);
cols = {st.normalMotion, st.slowMotion, st.stopped, st.protective};
for j = 1:4, b(j).FaceColor = cols{j}; end
set(ax,'YDir','reverse','YTick',1:nC,'YTickLabel',names,'TickLabelInterpreter','none', ...
    'XLim',[0 100],'YLim',[0.4 nC+0.6],'FontName','Helvetica','FontSize',8.5,'Box','off', ...
    'XGrid','on','GridColor',[0.80 0.80 0.78],'TickDir','out','Layer','top', ...
    'XColor',[0.32 0.32 0.31],'YColor',[0.32 0.32 0.31]);
xlabel(ax,'Share of the run time [%]','FontWeight','bold');
for i = 1:nC
    text(ax, 101.5, i, valueTxt{i}, 'FontSize',8.5,'Color',st.ink,'VerticalAlignment','middle');
end
lg = legend(ax, b, {'normal speed','slow','stopped (hold / wait)','protective stop'}, ...
    'Orientation','horizontal','Box','off','FontSize',8.5);
lg.Units = 'pixels';
lg.Position(1:2) = [330, hFig-75];
annotation(fig,'textbox',[0.02 0 0.96 1],'Units','pixels','Position',[20 hFig-45 1060 30], ...
    'String',ttl,'EdgeColor','none','FontSize',13,'FontWeight','bold','Interpreter','none');
if ~isempty(subttl)
    annotation(fig,'textbox',[0.02 0 0.96 1],'Units','pixels','Position',[20 hFig-62 1060 20], ...
        'String',subttl,'EdgeColor','none','FontSize',9,'Color',st.ink2,'Interpreter','none');
end
end

function saveFig(fig, fileName)
set(fig,'PaperPositionMode','auto');
print(fig, fileName, '-dpng', '-r200');
fprintf('saved %s.png\n', fileName);
end
