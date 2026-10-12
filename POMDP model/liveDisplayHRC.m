function h = liveDisplayHRC(h, prm, env, geo, out, tl, info)
% Live window of the virtual-robot tests: top view of the workstation (frame W)
% with the VIRTUAL robot, its zones and the REAL worker's arms from the camera,
% plus the decisions, the worker state and the message for the worker.
% h = liveDisplayHRC([], prm)          create the window (returns handles)
% h = liveDisplayHRC([], prm, panel)   draw it inside a panel of a combined window
% h = liveDisplayHRC(h, prm, env, geo, out, tl, info)   update it
%   info: t, sessionOn, estop, rxCount, age, pkt (newest camera message)
st = plotStyleHRC();
g = prm.geo;
if isempty(h)
    % h = liveDisplayHRC([], prm, parent): draw inside a panel (one combined window)
    parent = [];
    if nargin >= 3 && ~isempty(env) && ~isstruct(env) && isgraphics(env), parent = env; end
    if isempty(parent)
        h.fig = figure('Name','HRC live - real worker, virtual robot','NumberTitle','off', ...
            'Color','w','Position',[40 60 1380 780]);
        parent = h.fig;
    else
        h.fig = ancestor(parent,'figure');
    end
    inPanel = ~isequal(parent, h.fig);
    if inPanel                                   % combined window: top view above, text below
        posAx = [0.09 0.50 0.86 0.46]; posTx = [0.04 0.01 0.94 0.44]; keyFont = 8.5;
    else                                         % own window: top view left, text right
        posAx = [0.04 0.07 0.50 0.86]; posTx = [0.57 0.04 0.41 0.92]; keyFont = 9.5;
    end
    h.ax = axes('Parent',parent,'Position',posAx); hold(h.ax,'on');
    axis(h.ax,'equal'); grid(h.ax,'on'); box(h.ax,'on');
    A = prm.assemblyArea;                         % view: all places + the worker side of the table
    xs = [0 g.poseSolo(1) g.poseStorage(1) g.poseAssist(1) g.poseCollab(1) A.x];
    ys = [0 g.poseSolo(2) g.poseStorage(2) g.poseAssist(2) g.poseCollab(2) A.y];
    set(h.ax,'XLim',[min(xs)-0.35 max(xs)+0.45],'YLim',[min(ys)-0.30 max(ys)+0.30], ...
        'FontSize',9,'GridColor',[0.8 0.8 0.78]);
    xlabel(h.ax,'x [m]  (towards the worker)'); ylabel(h.ax,'y [m]  (worker''s right)');
    title(h.ax,'Top view of the workstation (frame W)','FontSize',12);
    rectangle(h.ax,'Position',[-0.08 -0.08 0.16 0.16],'FaceColor',[0.75 0.75 0.73],'EdgeColor','none');
    text(h.ax,0,-0.13,'robot base','HorizontalAlignment','center','FontSize',8,'Color',st.ink2);
    patch(h.ax,A.x([1 2 2 1]),A.y([1 1 2 2]),st.green,'FaceAlpha',0.15,'EdgeColor',st.green,'LineWidth',1.5);
    text(h.ax,A.x(2)-0.01,A.y(2)-0.03,'assembly area','HorizontalAlignment','right','FontSize',9, ...
        'FontWeight','bold','Color',st.green);
    pts = {g.poseSolo,'safe position'; g.poseStorage,'tool storage'; g.poseAssist,'handover point'; ...
        g.poseCollab,'collab. work point'};
    for i = 1:size(pts,1)
        p = pts{i,1};
        plot(h.ax,p(1),p(2),'s','MarkerSize',8,'MarkerEdgeColor',st.muted,'MarkerFaceColor',[0.93 0.93 0.91]);
        text(h.ax,p(1)+0.03,p(2)+0.03,pts{i,2},'FontSize',8,'Color',st.ink2);
    end
    th = linspace(0,2*pi,90);
    h.circle = th;
    h.zA = plot(h.ax,NaN,NaN,'--','Color',st.warning,'LineWidth',1.2);
    h.zC = plot(h.ax,NaN,NaN,'--','Color',st.serious,'LineWidth',1.2);
    h.zP = plot(h.ax,NaN,NaN,'-','Color',st.critical,'LineWidth',1.5);
    h.robot = plot(h.ax,NaN,NaN,'-','Color',[0.35 0.35 0.35],'LineWidth',7);
    h.tool = plot(h.ax,NaN,NaN,'o','MarkerSize',11,'MarkerFaceColor',[0.25 0.25 0.25],'MarkerEdgeColor','w');
    h.armL = plot(h.ax,NaN,NaN,'-o','Color',st.blue,'LineWidth',3,'MarkerSize',5,'MarkerFaceColor',st.blue);
    h.armR = plot(h.ax,NaN,NaN,'-o','Color',st.blue,'LineWidth',3,'MarkerSize',5,'MarkerFaceColor',st.blue);
    h.palm = plot(h.ax,NaN,NaN,'p','MarkerSize',15,'MarkerFaceColor',st.orange,'MarkerEdgeColor','w');
    legend(h.ax,[h.robot h.armR h.palm h.zP h.zC h.zA], {'robot (real when commanded)','worker arm (camera)', ...
        'hand closest to the robot','prohibited distance','collaborative zone','assistive zone'}, ...
        'Location','southwest','FontSize',8,'Box','off','AutoUpdate','off');
    h.tx = axes('Parent',parent,'Position',posTx,'Visible','off'); hold(h.tx,'on');
    set(h.tx,'XLim',[0 1],'YLim',[0 1]);
    labels = {'SESSION','MODE / PHASE','ROBOT ACTION','SAFETY','WORKER (camera)','FATIGUE MONITOR','CAMERA LINK'};
    y = [0.96 0.885 0.81 0.715 0.60 0.49 0.405];
    h.val = gobjects(1,numel(labels));
    for i = 1:numel(labels)
        text(h.tx,0,y(i),labels{i},'FontSize',9,'FontWeight','bold','Color',st.muted);
        h.val(i) = text(h.tx,0.27,y(i),'','FontSize',11,'Color',st.ink,'Interpreter','none', ...
            'VerticalAlignment','middle');
    end
    h.msgBox = rectangle(h.tx,'Position',[0 0.215 1 0.14],'FaceColor',[0.96 0.96 0.95],'EdgeColor','none');
    h.msg = text(h.tx,0.02,0.285,'','FontSize',13,'FontWeight','bold','Color',st.ink,'Interpreter','none');
    keys = {'G  START session        X  STOP session','P  PAUSE (rest)          R / ENTER  RESUME / continue', ...
        'T  bring me a tool       K  I took the tool','1 solo   2 assistive   3 collaborative', ...
        'E  emergency stop (on/off)     Esc  quit'};
    text(h.tx,0,0.165,'KEYS (click this window first)','FontSize',9,'FontWeight','bold','Color',st.muted);
    for i = 1:numel(keys)
        text(h.tx,0,0.165-0.032*i,keys{i},'FontSize',keyFont,'FontName','Consolas','Color',st.ink2,'Interpreter','none');
    end
    return
end
if ~isvalid(h.fig), return; end
% ---------------- robot and zones
r = env.robot;
set(h.robot,'XData',[r.base(1) r.elbow(1) r.tool(1)],'YData',[r.base(2) r.elbow(2) r.tool(2)]);
set(h.tool,'XData',r.tool(1),'YData',r.tool(2));
c = h.circle;
set(h.zP,'XData',r.tool(1)+prm.dProhibit*cos(c),'YData',r.tool(2)+prm.dProhibit*sin(c));
set(h.zC,'XData',r.tool(1)+prm.dCollabMax*cos(c),'YData',r.tool(2)+prm.dCollabMax*sin(c));
if isfield(prm,'assistZone') && prm.assistZone.enabled    % fixed circle around the handover point
    ac = prm.assistZone.centre; ar = prm.assistZone.radius;
    set(h.zA,'XData',ac(1)+ar*cos(c),'YData',ac(2)+ar*sin(c));
else
    set(h.zA,'XData',r.tool(1)+prm.dAssistMax*cos(c),'YData',r.tool(2)+prm.dAssistMax*sin(c));
end
% ---------------- worker arms
hs = {h.armL, h.armR}; sides = {'left','right'};
for i = 1:2
    P = NaN(4,3);
    if ~isempty(info.pkt) && isfield(info.pkt,'arms') && isfield(info.pkt.arms, sides{i})
        a = info.pkt.arms.(sides{i}); f = {'shoulder','elbow','wrist','palm'};
        for j = 1:4
            if isfield(a,f{j}) && numel(a.(f{j}))==3, P(j,:) = a.(f{j})(:)'; end
        end
    end
    set(hs{i},'XData',P(:,1),'YData',P(:,2));
end
if isfield(geo,'human') && isfield(geo.human,'palm') && all(isfinite(geo.human.palm)) && ~isempty(geo.arm)
    set(h.palm,'XData',geo.human.palm(1),'YData',geo.human.palm(2));
else
    set(h.palm,'XData',NaN,'YData',NaN);
end
% ---------------- text
act = actionDefinitionHRC();
nm = [{'tr  task layer (move / rest)'} strcat(act.short, {'  '}, lower(strrep(act.names,'_',' ')))];
modes = {'solo','assistive','collaborative'};
s = out.status;
lv = {'low','medium','high'}; at = {'focused','uncertain','distracted'};
zn = {'prohibited','collaborative','assistive','separated'}; rk = {'none','potential','active'};
if info.sessionOn, ses = 'ON'; else, ses = 'OFF  (press G to start)'; end
if info.estop, ses = [ses '   |   EMERGENCY STOP']; end
set(h.val(1),'String',ses,'Color',ifc(info.sessionOn && ~info.estop, st.good, st.critical),'FontWeight','bold');
set(h.val(2),'String',sprintf('%s  /  %s', modes{tl.mode+1}, tl.phase));
set(h.val(3),'String',sprintf('%s\nplanner: %s', nm{out.aF+1}, nm{out.aN+1}));
reason = out.supervisor.reason;
if ~isempty(out.adaptReason), reason = [out.adaptReason '  ' reason]; end
inA = {'no','yes'};
set(h.val(4),'String',sprintf('zone %s, risk %s, hand %.2f m, arm %.2f m, hand in assembly area: %s\n%s', ...
    pick(zn,geo.zone), pick(rk,geo.risk), geo.handInteractionDistance, geo.minUpperLimbClearance, ...
    inA{geo.inAssemblyArea+1}, reason), ...
    'Color',ifc(out.aF==10, st.critical, st.ink));
p = info.pkt;
if isempty(p)
    set(h.val(5),'String','no camera message yet');
else
    set(h.val(5),'String',sprintf('mental %s, physical %s, attention %s\nface %d, present %d, confidence %.2f', ...
        pick(lv,fld(p,'Fm')), pick(lv,fld(p,'Fp')), pick(at,fld(p,'attention')), ...
        fld(p,'face'), fld(p,'person'), fld(p,'conf')));
end
set(h.val(6),'String',sprintf('level %s, dialogue: %s', lv{s.level+1}, s.dialog), ...
    'Color',ifc(s.level==2, st.critical, st.ink));
trk = '';
if isfield(geo,'track'), trk = geo.track; end
set(h.val(7),'String',sprintf('%d messages, newest %.2f s old\n%s', info.rxCount, info.age, trk), ...
    'Color',ifc(info.age > prm.maxGeometryAge, st.critical, st.ink));
m = s.message;
col = [0.96 0.96 0.95];
if s.restRecommended, col = [0.99 0.94 0.80]; end
if s.restActive, col = [0.86 0.95 0.91]; end
if isempty(m), m = 'no message for the worker'; end
set(h.msg,'String',textwrap({m}, 52));
set(h.msgBox,'FaceColor',col);
title(h.ax,sprintf('Top view of the workstation (frame W)   t = %.1f s', info.t),'FontSize',12);
drawnow limitrate
end

function c = ifc(cond, a, b)
if cond, c = a; else, c = b; end
end

function s = pick(names, v)
if isempty(v) || isnan(v), s = 'unknown'; else, s = names{v+1}; end
end

function v = fld(p, f)
v = NaN;
if isfield(p,f) && ~isempty(p.(f)), v = double(p.(f)(1)); end
end
