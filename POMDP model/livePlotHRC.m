function lp = livePlotHRC(lp, prm, r)
% Real-time plot of the live test (same layout and colors as the simulation
% figures, plotStyleHRC). The time axis scrolls: the last lp.window seconds.
%
% lp = livePlotHRC([], prm)        create the window (figure 'HRC live - signals')
% lp = livePlotHRC([], prm, panel) draw inside a panel of a combined window
% lp = livePlotHRC(lp, prm, r)     add one decision epoch and redraw
%   r: t, Fp, Fm, A (observed levels, NaN = unknown), bFp, bFm, bA (robot belief),
%      handDist, clearance, Z, R, verified, aN, aF, motion (-1 protective stop,
%      0 stopped, 1 slow, 2 normal), level (fatigue monitor), dialogCode
%      (0 none, 1 warning, 2 continued, 3 rest), button (0 none, 1 RESUME,
%      2 PAUSE, 4 START, 5 STOP), sessionOn, inAssembly, mode (0 solo,
%      1 assistive, 2 collaborative), robotSpeed (tool [m/s]), mentalScore,
%      physicalF (sensor signals 0..1), awayS (time looking away [s]),
%      cmdSpeed (commanded joint speed [deg/s]), jointSpeed (measured [deg/s]),
%      conf (confidence of the human-state estimate 0..1), cause (speedCauseHRC code)
% Panels: 1 monitor strips (fatigue level, attention level, dialogue, worker
% button, session, hand in assembly area), 2 task mode, 3 fatigue sensor
% signals with their level thresholds, 4 attention sensor signal (time looking
% away, 2 s distraction limit), 5 distances, 6 robot speed (commanded and
% measured joint speed), 7 cause of a speed reduction (speedCauseHRC), 8 safety
% strips, 9 robot action.
% Background: yellow = rest warning, green = rest, red = protective stop.
st = plotStyleHRC();
% action rows of this plot: the common order plus 11 = wait at the safe position
% (liveHRC shows "retreat" this way while the robot already stands there)
st.actionOrder = [10 7 4 11 0 9 8 3 6 2 5 1];
st.actionLabels = [st.actionLabels(1:3) {'wait at safe position'} st.actionLabels(4:end)];
NC = 29;                                         % buffer columns (see "buffer" below)
if isempty(lp)
    lp.window = 60;                              % seconds shown
    lp.n = 0;
    lp.buf = zeros(0, NC);
    ttl = 'Live signals - real worker, robot';
    subttl = 'yellow = rest warning   |   green = worker resting   |   red = protective stop   |   last 60 s';
    wts = [1.2 0.42 0.95 0.7 0.85 0.8 1.15 0.6 1.75];   % monitor, mode, fatigue, attention, distance, speed, cause, safety, action
    if nargin >= 3 && ~isempty(r) && ~isstruct(r) && isgraphics(r)
        fig = ancestor(r,'figure');
        lp.ax = panelStack(r, wts, ttl, subttl);
    else
        fig = figure('Name','HRC live - signals','NumberTitle','off');
        lp.ax = st.stack(fig, wts, ttl, subttl);
    end
    lp.fig = fig;
    ax = lp.ax;
    for i = 1:numel(ax), hold(ax(i),'on'); end
    lp.shade = gobjects(numel(ax),3);
    lp.l5 = gobjects(1,4);
    % ---- background shading (one patch per kind, faces rebuilt each update)
    shadeCol = {[0.99 0.94 0.80], [0.86 0.95 0.91], st.stopShade};
    for i = 2:numel(ax)
        for j = 1:3
            lp.shade(i,j) = patch(ax(i),'Faces',[],'Vertices',zeros(0,2),'FaceColor',shadeCol{j}, ...
                'EdgeColor','none','HandleVisibility','off');
        end
    end
    % ---- 1 monitor strips
    lp.img1 = image(ax(1),'XData',[0 1],'YData',[1 6],'CData',ones(6,1,3));
    set(ax(1),'YDir','normal','YLim',[0.5 6.5],'YTick',1:6,'YTickLabel', ...
        fliplr({'fatigue level','attention','dialogue','worker button','session','hand in assembly'}));
    ylabel(ax(1),'Monitor');
    hp = gobjects(1,6);
    cl = {st.good, st.warning, st.critical, st.green, st.normalMotion, [0.76 0.76 0.72]};
    for i = 1:6, hp(i) = patch(ax(1),NaN,NaN,cl{i},'EdgeColor','none'); end
    st.legend(ax(1), hp, {'low | focused','medium | uncertain','high | distracted | PAUSE', ...
        'rest / session on / in area','RESUME / START','unknown'});
    % ---- 2 task mode
    a = ax(2);
    lp.lm = stairs(a,NaN,NaN,'Color',[0.15 0.15 0.15],'LineWidth',st.lw);
    set(a,'YLim',[-0.4 2.4],'YTick',0:2,'YTickLabel',{'solo','assistive','collaborative'});
    ylabel(a,'Task mode');
    % ---- 3 fatigue sensor signals (0..1) and their level thresholds
    a = ax(3);
    lp.lf = [plot(a,NaN,NaN,'-','Color',st.blue,'LineWidth',st.lw), plot(a,NaN,NaN,'-','Color',st.orange,'LineWidth',st.lw)];
    thF = [plot(a,[0 1e7],[0.3 0.3],':','Color',st.blue,'LineWidth',1), plot(a,[0 1e7],[0.6 0.6],'--','Color',st.blue,'LineWidth',1), ...
           plot(a,[0 1e7],[0.4 0.4],':','Color',st.orange,'LineWidth',1), plot(a,[0 1e7],[0.7 0.7],'--','Color',st.orange,'LineWidth',1)];
    lp.lq = plot(a,NaN,NaN,'-','Color',[0.55 0.55 0.55],'LineWidth',1);                 % estimate confidence
    lqt = plot(a,[0 1e7],[prm.minHumanConfidence prm.minHumanConfidence],'-.','Color',[0.55 0.55 0.55],'LineWidth',0.8);
    set(a,'YLim',[0 1.05],'YTick',0:0.25:1);
    ylabel(a,{'Fatigue','signal'});
    st.legend(a, [lp.lf thF([1 2 3 4]) lp.lq lqt], {'physical index f (camera)','mental score (camera)', ...
        'physical medium 0.3','physical high 0.6','mental medium 0.4','mental high 0.7', ...
        'estimate confidence', sprintf('confidence limit %.1f', prm.minHumanConfidence)});
    % ---- 4 attention sensor signal: time looking away, 2 s distraction limit
    a = ax(4);
    lp.la = plot(a,NaN,NaN,'-','Color',st.green,'LineWidth',st.lw);
    lpd = plot(a,[0 1e7],prm.distractionLimit*[1 1],'--','Color',st.critical,'LineWidth',1);
    set(a,'YLim',[0 4]);
    ylabel(a,{'Looking','away [s]'});
    st.legend(a, [lp.la lpd], {'time looking away (camera)', sprintf('distraction limit %.0f s',prm.distractionLimit)});
    % ---- 5 distances
    a = ax(5);
    lp.l3 = [plot(a,NaN,NaN,'Color',st.blue,'LineWidth',st.lw), plot(a,NaN,NaN,'Color',st.orange,'LineWidth',st.lw)];
    th = [prm.dProhibit prm.dCollabMax prm.dAssistMax];
    thc = {st.critical, st.serious, st.warning};
    nth = 3 - (isfield(prm,'assistZone') && prm.assistZone.enabled);   % fixed assistive zone: no distance line
    lth = gobjects(1,nth);
    for i = 1:nth
        lth(i) = plot(a,[0 1e7],[th(i) th(i)],'--','Color',thc{i},'LineWidth',1);
    end
    set(a,'YLim',[0 max(1.0, th(nth) + 0.3)]);
    ylabel(a,'Distance [m]');
    thn = {sprintf('prohibited %.2f m',th(1)), sprintf('collaborative %.2f m',th(2)), sprintf('assistive %.2f m',th(3))};
    st.legend(a, [lp.l3 lth], [{'hand \rightarrow robot tool','min. upper-limb clearance'} thn(1:nth)]);
    % ---- 6 robot speed: commanded (decision) and measured joint speed
    a = ax(6);
    lp.ls = [stairs(a,NaN,NaN,'--','Color',[0.10 0.10 0.10],'LineWidth',1.4), ...
             plot(a,NaN,NaN,'-','Color',[0.45 0.25 0.60],'LineWidth',st.lw)];
    set(a,'YLim',[0 35],'YTick',0:10:30);
    ylabel(a,{'Joint speed','[deg/s]'});
    st.legend(a, lp.ls, {'commanded (decision)','measured (robot)'});
    % ---- 7 cause of a speed reduction (speedCauseHRC)
    a = ax(7);
    [cc, cn] = causeStyle();
    lp.lc = gobjects(1,8);
    for c = 1:8
        lp.lc(c) = plot(a,NaN,NaN,'-','Color',cc(c,:),'LineWidth',6);
    end
    set(a,'YLim',[0.4 8.6],'YTick',1:8,'YTickLabel',cn,'FontSize',8);
    ylabel(a,{'Speed','reduced by'});
    % ---- 8 safety strips
    lp.img4 = image(ax(8),'XData',[0 1],'YData',[1 3],'CData',ones(3,1,3));
    set(ax(8),'YDir','normal','YLim',[0.5 3.5],'YTick',1:3,'YTickLabel',fliplr({'hand zone Z','risk R','verified'}));
    ylabel(ax(8),'Safety');
    hp = gobjects(1,5);
    cl = {st.good, st.warning, st.serious, st.critical, [0.76 0.76 0.72]};
    for i = 1:5, hp(i) = patch(ax(8),NaN,NaN,cl{i},'EdgeColor','none'); end
    st.legend(ax(8), hp, {'separated | no risk | verified','assistive | potential','collaborative', ...
        'prohibited | active | NOT verified','unknown'});
    % ---- 9 actions
    a = ax(9);
    lp.l5n = stairs(a,NaN,NaN,'Color',[0.62 0.62 0.60],'LineWidth',1);
    cols = {st.normalMotion, st.slowMotion, st.stopped, st.protective};
    for c = 1:4
        lp.l5(c) = plot(a,NaN,NaN,'-','Color',cols{c},'LineWidth',6);
    end
    set(a,'YLim',[0.3 numel(st.actionOrder)+0.7],'YTick',1:numel(st.actionOrder),'YTickLabel',st.actionLabels);
    ylabel(a,'Robot action');
    st.legend(a, [lp.l5 lp.l5n], {'normal speed','slow','stopped (hold / wait)','protective stop','planner suggestion'});
    st.finish(ax, [0; 1]);
    set(ax([7 9]),'FontSize',8);
    set(ax([1 8]),'XGrid','off','YGrid','off');
    return
end
if ~isvalid(lp.fig), return; end
% ---------------- buffer: 1 t, 2-4 Fp Fm A, 5-7 beliefs, 8 handDist, 9 clearance, 10 Z, 11 R, 12 verified,
% 13 aN, 14 aF, 15 motion, 16 level, 17 dialog, 18 button, 19 session, 20 inAssembly, 21 mode, 22 tool speed,
% 23 mental score, 24 physical f, 25 time away, 26 commanded joint speed, 27 measured joint speed,
% 28 estimate confidence, 29 cause of a speed reduction
row = [r.t r.Fp r.Fm r.A r.bFp r.bFm r.bA r.handDist r.clearance r.Z r.R r.verified ...
       r.aN r.aF r.motion r.level r.dialogCode r.button r.sessionOn r.inAssembly fieldOr(r,'mode',NaN) ...
       fieldOr(r,'robotSpeed',NaN) fieldOr(r,'mentalScore',NaN) fieldOr(r,'physicalF',NaN) fieldOr(r,'awayS',NaN) ...
       fieldOr(r,'cmdSpeed',NaN) fieldOr(r,'jointSpeed',NaN) fieldOr(r,'conf',NaN) fieldOr(r,'cause',NaN)];
lp.buf(end+1,:) = row;
keep = lp.buf(:,1) >= r.t - lp.window - 1;
lp.buf = lp.buf(keep,:);
B = lp.buf; t = B(:,1); dt = prm.dt; n = numel(t);
x0 = max(r.t - lp.window, 0); x1 = max(r.t, x0 + 1) + dt;
% ---------------- 1 monitor strips (RGB image, one column per step)
gray = [0.94 0.94 0.93]; unk = [0.76 0.76 0.72];
lvlC = [st.good; st.warning; st.critical];
dlgC = [gray; st.warning; st.slowMotion; st.green];
btnC = containers.Map({0,1,2,3,4,5},{[1 1 1], st.normalMotion, st.critical, st.normalMotion, st.normalMotion, st.critical});
C = ones(6, n, 3);
for k = 1:n
    C(6,k,:) = lvlC(B(k,16)+1,:);
    C(5,k,:) = pickC(lvlC, B(k,4), unk);         % attention level: focused / uncertain / distracted
    C(4,k,:) = dlgC(B(k,17)+1,:);
    C(3,k,:) = btnC(B(k,18));
    C(2,k,:) = ifc(B(k,19)==1, st.green, gray);
    C(1,k,:) = ifc(B(k,20)==1, st.green, gray);
end
set(lp.img1,'XData',[t(1) t(end)] + dt/2,'CData',C);
% ---------------- 2 task mode
set(lp.lm,'XData',t,'YData',B(:,21));
% ---------------- 3 fatigue sensor signals
set(lp.lf(1),'XData',t,'YData',B(:,24));
set(lp.lf(2),'XData',t,'YData',B(:,23));
set(lp.lq,'XData',t,'YData',B(:,28));
% ---------------- 4 attention sensor signal (y-axis grows with the longest look-away shown)
set(lp.la,'XData',t,'YData',B(:,25));
amax = max(B(:,25),[],'omitnan');
if isfinite(amax), set(lp.ax(4),'YLim',[0 max(4, 1.15*amax)]); end
% ---------------- 5 distances
set(lp.l3(1),'XData',t,'YData',B(:,8));
set(lp.l3(2),'XData',t,'YData',B(:,9));
% ---------------- 6 robot speed
set(lp.ls(1),'XData',t,'YData',B(:,26));
set(lp.ls(2),'XData',t,'YData',B(:,27));
% ---------------- 7 cause of a speed reduction
for c = 1:8
    idx = find(B(:,29)==c);
    X = [t(idx) t(idx)+dt NaN(numel(idx),1)]'; Y = [c*ones(numel(idx),1) c*ones(numel(idx),1) NaN(numel(idx),1)]';
    set(lp.lc(c),'XData',X(:),'YData',Y(:));
end
% ---------------- 8 safety strips
zC = st.zoneColors; rC = st.riskColors;
C = ones(3, n, 3);
for k = 1:n
    C(3,k,:) = pickC(zC, B(k,10), unk);
    C(2,k,:) = pickC(rC, B(k,11), unk);
    C(1,k,:) = ifc(B(k,12)==1, st.good, st.critical);
end
set(lp.img4,'XData',[t(1) t(end)] + dt/2,'CData',C);
% ---------------- 9 actions
lev = @(a) arrayfun(@(x) find(st.actionOrder==x,1), a);
yN = lev(B(:,13)); yF = lev(B(:,14));
set(lp.l5n,'XData',t,'YData',yN);
lvl = [2 1 0 -1];
for c = 1:4
    idx = find(B(:,15)==lvl(c));
    X = [t(idx) t(idx)+dt NaN(numel(idx),1)]'; Y = [yF(idx) yF(idx) NaN(numel(idx),1)]';
    set(lp.l5(c),'XData',X(:),'YData',Y(:));
end
% ---------------- background shading
masks = {B(:,17)==1, B(:,17)==3, B(:,14)==10};
for i = 2:numel(lp.ax)
    yl = get(lp.ax(i),'YLim');
    for j = 1:3
        idx = find(masks{j});
        if isempty(idx)
            set(lp.shade(i,j),'Faces',[],'Vertices',zeros(0,2));
        else
            m = numel(idx);
            V = [t(idx) yl(1)*ones(m,1); t(idx)+dt yl(1)*ones(m,1); t(idx)+dt yl(2)*ones(m,1); t(idx) yl(2)*ones(m,1)];
            F = [(1:m)' (m+1:2*m)' (2*m+1:3*m)' (3*m+1:4*m)'];
            set(lp.shade(i,j),'Vertices',V,'Faces',F);
        end
    end
end
set(lp.ax,'XLim',[x0 x1]);
end

function ax = panelStack(parent, weights, ttl, subttl)
% same stacked layout as plotStyleHRC/stack, but inside a panel (no new
% window, no resizing; titles as text because annotations need a figure)
left = 0.20; width = 0.56; top = 0.905; bottom = 0.05; gap = 0.014;
n = numel(weights);
hh = (top - bottom - gap*(n-1))*weights/sum(weights);
ax = gobjects(n,1);
y = top;
for i = 1:n
    y = y - hh(i);
    ax(i) = axes('Parent',parent,'Position',[left y width hh(i)]);
    y = y - gap;
end
tAx = axes('Parent',parent,'Position',[0.02 0.91 0.96 0.085],'Visible','off', ...
    'XLim',[0 1],'YLim',[0 1],'HitTest','off');
text(tAx,0,0.62,ttl,'FontSize',13,'FontWeight','bold','Interpreter','none');
text(tAx,0,0.12,subttl,'FontSize',9,'Color',[0.32 0.32 0.31],'Interpreter','none');
end

function [c, n] = causeStyle()
% colours and names of the causes 1..8 (speedCauseHRC; 0 = none is not drawn)
c = [0.90 0.60 0.00; 0.60 0.35 0.10; 0.80 0.15 0.15; 0.55 0.30 0.70; ...
     0.35 0.50 0.70; 0.55 0.55 0.55; 0.00 0.55 0.55; 0.75 0.75 0.72];
n = {'fatigue level','fatigue + distance','distance','attention','low confidence','sensing','planner (belief)','task / worker'};
end

function v = fieldOr(s, f, d)
if isfield(s, f), v = s.(f); else, v = d; end
end

function c = ifc(cond, a, b)
if cond, c = a; else, c = b; end
end

function c = pickC(cols, v, unk)
if isnan(v), c = unk; else, c = cols(v+1,:); end
end
