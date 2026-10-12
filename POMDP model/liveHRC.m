clc;close all;clear stateDefinitionHRC rd
% LIVE TEST: REAL worker in front of the camera; robot = the REAL robot's measured pose when the camera
% program sends it (workstation calibration on the real robot, see realCameraObservationAdapter), else VIRTUAL
%
% Start order
%   1. once per workstation set-up:  python workspace_calibration.py   (folder mental_fatigue_algorithm):
%      drag the assembly area (also measures the table), click the bottle
%      (= robot: base at its foot, safe position at its tip), click the tissue
%      packet (= tool storage); the handover point is set automatically
%   2. python main.py   (camera program; it sends the worker state + arms to this script)
%      in its window: U slides, S baseline, 1/2/3 station calibration, Y physical baseline
%   3. run this script, click its window and press G (START)
% SCENARIO RECORDING: everything from G (START) to X (STOP) is one scenario.
% At X everything is saved in results\scenario_<mode>_<date>\ (scenarioReportHRC):
% journal figure (.pdf/.png/.fig), data (.mat, _data.csv), metrics for the paper
% (_metrics.csv) and the event / action list (_events.csv). To re-make them later
% without a new experiment: scenarioReportHRC('results\...\scenario_....mat');
% table of all runs: scenarioSummaryHRC('results').
%
% Every 0.5 s: newest camera message -> observation (real worker + virtual robot)
% -> task layer (taskLayerLiveHRC) -> hrcStepHRC (belief, QMDP planner, fatigue
% monitor, fatigue adaptation, safety supervisor) -> the virtual robot moves
% (syntheticEnvironmentHRC) -> live window + log.
% Two windows: "HRC live" (top view, decisions, keys) and "HRC live - signals"
% (real-time plot of the last 60 s, same layout as the simulation figures).
% FATIGUE WARNING: when the sustained fatigue level becomes high, a pop-up window
% (fatigueWarningPopupHRC) asks the worker to rest; in collaborative mode the robot
% stops until the answer: ENTER = continue working, P = rest (ENTER again when back).
% Keys (window "HRC live"): G START, X STOP session, P PAUSE (rest), R RESUME,
%   T bring me a tool, K I took the tool, 1 solo, 2 assistive, 3 collaborative,
%   E emergency stop on/off, Esc quit.
% Needs PHRC.mat / VHRC.mat (valueIterationHRC) and MATLAB R2020b or newer.

% ======================= settings you can change =========================
port      = 5005;        % UDP port (config.json of the camera program: pomdp_bridge.port)
startMode = 1;           % 0 solo, 1 assistive, 2 collaborative
participant = 'P01';     % participant id: stored with every recorded scenario (analyzeSessionsHRC groups by it)
condition = 'adaptive';  % 'adaptive' = proposed method; 'fixed' = BASELINE: the same safety supervisor (distance,
                         % attention, sensing stops) but NO speed reduction from fatigue, estimate confidence or the
                         % planner's fatigue belief (approach at normal speed, solo / collaborative normal speed)
logFile   = ['liveLog_' char(datetime('now','Format','yyyyMMdd_HHmmss')) '.csv'];
handOnlySafety = true;   % true: safety from the HAND (palm) only when the camera cannot see the
                         % shoulder / elbow / wrist (they are estimated from the palm, see
                         % realCameraObservationAdapter); false: a complete measured arm is required
palmOnlyClearance = true; % true: arm-robot clearance from the PALM only (elbow, forearm and upper
                         % arm ignored); false: whole arm (upper arm, forearm, hand)
actionFile = 'action.md'; % robot command file: one row per decision epoch (0.5 s), rewritten at
                         % every start of this script; the robot interface reads its LAST row
oneWindow = true;        % true: live view and signals side by side in ONE window; false: two windows
splitX    = 0.45;        % one window: width share of the left part (top view + decisions)
figWidth  = 3.5;         % scenario figure width [in]: 3.5 IEEE single column, 7.16 double column
figHeight = 7.0;         % scenario figure height [in] (7 panels: mode, fatigue, attention, distance, speed, cause, action)
commandRealRobot = true; % true: the decided action is written to actions.md -> robot_code/robot_code.py moves the
                         % REAL robot (action files tasks/task<N>.txt, robotActionHRC). The robot is commanded only
                         % after G (START) and only while the camera program sends the real robot pose; otherwise
                         % 11 (hold) before START, 10 (stop) without the real pose, at Esc and on an error.
actionsFile = fullfile(fileparts(mfilename('fullpath')), '..', 'actions.md');
recordVideo = true;      % true: VIDEO of the experiment from G (START) / R (RESUME) until P (PAUSE) / X (STOP) / Esc:
                         % the live window(s) here + the camera (main.py) -> videos\session_<date>\experiment_video.mp4
videoRoot = fullfile(fileparts(mfilename('fullpath')), 'videos');
videoComposer = fullfile(fileparts(mfilename('fullpath')), '..', 'mental_fatigue_algorithm', 'make_experiment_video.py');
% =========================================================================

prm = stateDefinitionHRC();
prm.handOnlySafety = handOnlySafety;
prm.palmOnlyClearance = palmOnlyClearance;
act = actionDefinitionHRC();
if prm.calibrated
    fprintf('workstation calibration: %s\n', prm.calibrationFile);
else
    warning('No workspace_calibration.json in this folder: run workspace_calibration.py first (default geometry used).');
end
if isfield(prm,'fixedLayout')
    fprintf('fixed workstation layout from fixedLayoutHRC.m (places of the calibration file not used)\n');
end
actionFileInit(actionFile, act);
fprintf('robot commands are written to %s\n', actionFile);
fprintf('loading the planner tables ...\n');
load('PHRC'); load('VHRC');
[R,MK] = rewardHRC();
N = NextStateHRC();
T = TransProbHRC();
tab = struct('P',P,'V',V,'N',N,'T',T,'R',R,'MK',MK);
prmEnv = prm;                                    % virtual robot only: no simulated worker events
f = fieldnames(prmEnv.scenario);
for i = 1:numel(f), prmEnv.scenario.(f{i}) = zeros(0,2); end
env = syntheticEnvironmentHRC([], prmEnv);       % robot starts at its safe position
[~, rd] = cameraReaderHRC([], port);       % open the receiver (first output: no message yet)
mem = struct('autoStart', false);                % the planner waits for START (G)
[~, tl] = taskLayerLiveHRC(startMode);   % first output (task command) is empty at initialisation
ad = [];
if oneWindow                                     % both views side by side in ONE window
    figAll = figure('Name',['HRC live - real worker, ' repmat('real robot',1,commandRealRobot) repmat('virtual robot',1,~commandRealRobot)],'NumberTitle','off', ...
        'Color','w','Units','normalized','OuterPosition',[0 0.04 1 0.96]);
    pL = uipanel('Parent',figAll,'Units','normalized','Position',[0 0 splitX 1], ...
        'BorderType','none','BackgroundColor','w');
    pR = uipanel('Parent',figAll,'Units','normalized','Position',[splitX 0 1-splitX 1], ...
        'BorderType','none','BackgroundColor','w');
    h = liveDisplayHRC([], prm, pL);             % top view + decisions + keys (left)
    lp = livePlotHRC([], prm, pR);               % real-time signals (right)
else
    h = liveDisplayHRC([], prm);
    lp = livePlotHRC([], prm);                   % real-time signal plot (second window)
end
dialogs = {'none','warning','continued','rest'};
figure(h.fig);
setappdata(h.fig,'keys',{});
set(h.fig,'WindowKeyPressFcn',@(src,e) setappdata(src,'keys',[getappdata(src,'keys') {e.Key}]));
estop = 0; prevSt = []; latched = 0; k = 0; realSeen = -1;
pw = [];                                         % fatigue warning pop-up (fatigueWarningPopupHRC)
robotCmd = -1; robotScale = 1; commanding = false; cmdSpeed = 0;
spdTab = robotSpeedsHRC();                       % commanded joint speed of each action file [deg/s]
lastReset = NaN; lastCause = -1;                 % last speed-reduction cause printed (speedCauseHRC)                                 % camera program fatigue-reset counter (keys F, Z Z)               % real robot: last action written, commanded since START
vid = []; vidPart = 0;                           % experiment video (experimentVideoHRC)
if commandRealRobot
    robotCmd = writeRobotAction(actionsFile, 11, robotCmd, 0, 'liveHRC started: hold until START (G)');
end
logRows = {};
rec = [];                                        % scenario being recorded (G ... X)
recCols = {'t','Fp','Fm','A','dHand','clearance','aF','aN','motion','mode','dialog','verified', ...
    'requests','deliveries','button','zone','risk','speed','age','rxCount','armEst','inArea','conf', ...
    'level','aFraw','aNraw','speedScale','realRobot','mentalScore','physicalF','awayS','cmdSpeed','jointSpeed','cause'};
fprintf('listening on UDP port %d - click the live window and press G to start\n', port);
fprintf('participant %s | condition %s%s\n', participant, upper(condition), ...
    repmat(' (BASELINE: no fatigue / confidence speed adaptation)', 1, strcmp(condition, 'fixed')));
running = true;
try
while running && isvalid(h.fig)
    loopStart = tic;
    k = k + 1;
    % ---------------- keys
    keys = getappdata(h.fig,'keys'); setappdata(h.fig,'keys',{});
    btn = 0; cmd = struct('mode',-1,'request',false,'taken',false); keyTxt = strjoin(keys,' ');
    for i = 1:numel(keys)
        switch lower(keys{i})
            case 'g', btn = 4;
            case 'x', btn = 5;
            case 'p', btn = 2;
            case 'r', btn = 1;
            case 'return'                        % ENTER answers the fatigue warning / ends the rest
                if ~isempty(prevSt) && any(strcmp(prevSt.dialog, {'warning','rest'})), btn = 1; end
            case 't', cmd.request = true;
            case 'k', cmd.taken = true;
            case {'1','numpad1'}, cmd.mode = 0;
            case {'2','numpad2'}, cmd.mode = 1;
            case {'3','numpad3'}, cmd.mode = 2;
            case 'e', estop = ~estop;
            case 'escape', running = false;
        end
    end
    % ---------------- observation: newest camera message + virtual robot
    [pkt, rd] = cameraReaderHRC(rd);
    tNow = toc(rd.t0);
    [O, ev, geo, ad] = realCameraObservationAdapter(pkt, rd.lastRx, tNow, env, prm, ad);
    if geo.realRobot ~= realSeen                 % report when the robot source changes
        realSeen = geo.realRobot;
        if realSeen, fprintf('[%6.1f s] distances to the REAL robot (pose from the camera program)\n', tNow);
        else, fprintf('[%6.1f s] distances to the VIRTUAL robot - no real robot pose: %s\n', tNow, geo.robotStatus); end
    end
    if ~geo.verified && mod(k,4)==0              % every 2 s: why the safety geometry is not verified
        fprintf('[%6.1f s] safety NOT verified: %s | %s\n', tNow, geo.unverifiedReason, geo.track);
    end
    if commandRealRobot && commanding && geo.realRobot
        env.tool = geo.robot.tool;               % the task layer follows the REAL robot it commands
        env.robot = geo.robot;
    end
    O(10) = estop;
    O(18) = btn;
    % ---------------- task layer and one decision cycle of the framework
    [tc, tl] = taskLayerLiveHRC(tl, env, prevSt, cmd, geo.handInteractionDistance, latched, prm);
    % ---------------- feedback for T / K when they cannot be used now
    modeNames = {'SOLO','ASSISTIVE','COLLABORATIVE'};
    if cmd.request
        if tl.mode ~= 1
            fprintf('[%6.1f s] T ignored: "bring me a tool" works in ASSISTIVE mode only (now %s): press 2 first\n', ...
                tNow, modeNames{tl.mode+1});
        elseif tl.pendingRequest
            fprintf('[%6.1f s] T noted: the robot fetches the tool as soon as it waits at its safe position (now %s)\n', ...
                tNow, tl.phase);
        elseif ~strcmp(tl.phase, 'FETCH')
            fprintf('[%6.1f s] T ignored: a tool is already being handed over (phase %s)\n', tNow, tl.phase);
        else
            fprintf('[%6.1f s] T: the robot fetches a tool from the storage\n', tNow);
        end
    end
    if cmd.taken && ~any(strcmp(tl.phase, {'HOLD','RETURN'}))
        fprintf('[%6.1f s] K ignored: it confirms the tool only while the robot holds it at the handover point (now %s mode, %s)\n', ...
            tNow, modeNames{tl.mode+1}, tl.phase);
    end
    O(5) = tc.instruction;
    % ---------------- manual fatigue reset in the camera program (F, Z Z): restart the fatigue monitor and belief
    fr = geo.sig.fatigueReset;
    if isfinite(fr) && isfinite(lastReset) && fr ~= lastReset
        mem.b = [];
        if isfield(mem,'mon') && isstruct(mem.mon) && isfield(mem.mon,'par'), mem.mon = struct('par',mem.mon.par); else, mem.mon = []; end
        fprintf('[%6.1f s] fatigue RESET in the camera program: fatigue monitor and belief restarted (low)\n', tNow);
    end
    if isfinite(fr), lastReset = fr; end
    [aF, out, mem] = hrcStepHRC(O, geo, ev, tc, mem, tab);
    prevSt = out.status;
    pw = fatigueWarningPopupHRC(pw, prevSt, tc.mode, h.fig);   % pop-up while the fatigue warning / rest is active
    latched = ~isempty(mem.sup) && mem.sup.latched;
    % ---------------- approach speed by fatigue (handover approach, collaborative approach)
    lvl = prevSt.level; if isnan(lvl), lvl = 1; end
    apScale = 1;
    if strcmp(tl.phase, 'APPROACH') || (strcmp(tl.phase, 'MOVE') && tc.mode == 2)
        apScale = prm.approachSpeedScale(min(max(round(lvl), 0), 2) + 1);
    end
    % ---------------- baseline condition: remove fatigue / confidence / belief speed reductions (safety stays)
    if strcmp(condition, 'fixed')
        supC = ''; if ~isempty(mem.sup) && isfield(mem.sup,'cause'), supC = mem.sup.cause; end
        c0 = speedCauseHRC(aF, out.aN, out, tl.phase, tc.mode, apScale, supC, O(7));
        if any(c0 == [1 2 5 7])
            if aF == 6 || (aF == 7 && contains(out.adaptReason,'fatigue')), aF = 5; elseif aF == 2, aF = 1; end
            out.speedScale = 1; out.speedReason = ''; out.adaptReason = '';
            out.supervisor.reason = '';
        end
        apScale = 1;
    end
    % ---------------- real robot: action file number -> actions.md -> robot_code.py
    if commandRealRobot
        if btn == 4, commanding = true; end      % G: START
        if ~commanding
            n = 11; why = 'waiting for START (G)';
        elseif ~geo.realRobot
            n = 10; why = 'NO real robot pose from the camera program: robot stopped';
        else
            n = robotActionHRC(aF, tl.phase, tc.mode, prevSt.robotGoSafe, env.tool, prm.geo);
            why = sprintf('aF %d, phase %s', aF, tl.phase);
        end
        scaleOut = 1;
        if any(n == [1 2]), scaleOut = out.speedScale; end   % solo: fatigue / distance speed factor
        if n == 3 || (n == 5 && strcmp(tl.phase, 'MOVE'))     % approach: normal speed scaled by the fatigue level
            scaleOut = apScale;
            if apScale < 1, why = sprintf('%s, approach speed x%.2f (fatigue level %d)', why, apScale, prevSt.level); end
        end
        if ~isempty(out.speedReason), why = [why ', ' out.speedReason]; end
        [robotCmd, robotScale] = writeRobotAction(actionsFile, n, robotCmd, tNow, why, scaleOut, robotScale);
        cmdSpeed = spdTab(n + 1) * robotScale;       % joint speed the robot program commands now [deg/s]
    end
    % ---------------- cause of a speed reduction (speedCauseHRC), printed when it changes
    supCause = '';
    if ~isempty(mem.sup) && isfield(mem.sup,'cause'), supCause = mem.sup.cause; end
    [causeCode, causeLabel] = speedCauseHRC(aF, out.aN, out, tl.phase, tc.mode, apScale, supCause, O(7));
    if causeCode ~= lastCause && prevSt.sessionOn
        if causeCode == 0
            fprintf('[%6.1f s] speed: not reduced\n', tNow);
        else
            fprintf('[%6.1f s] speed REDUCED by: %s  (%s%s%s)\n', tNow, upper(causeLabel), out.supervisor.reason, ...
                repmat([' ' out.adaptReason], 1, ~isempty(out.adaptReason)), repmat([' ' out.speedReason], 1, ~isempty(out.speedReason)));
        end
        lastCause = causeCode;
    end
    % ---------------- virtual robot moves
    env.taskTarget = out.target;
    env.speedScale = out.speedScale * apScale;   % solo fatigue / distance factor, approach factor
    if commandRealRobot && commanding && ~geo.realRobot
        env.speedScale = 0;                      % real robot pose lost: the virtual robot FREEZES at the last real pose
    end                                          % (it must not move on and advance the task phase by itself)
    env = syntheticEnvironmentHRC(env, prmEnv, k, tc.mode, aF);
    % ---------------- display and log
    info = struct('t',tNow,'sessionOn',prevSt.sessionOn,'estop',estop,'rxCount',rd.count, ...
        'age',tNow - rd.lastRx,'pkt',pkt);
    envDisp = env;
    if geo.realRobot, envDisp.robot = geo.robot; end   % top view: the real robot
    h = liveDisplayHRC(h, prm, envDisp, geo, out, tl, info);
    Bm = reshape(out.b,3,3,3);                   % belief (A, Fm, Fp) -> expected levels
    mot = env.commandedMotion; if aF==10, mot = -1; end
    % for the plots: "retreat" while the robot already stands at its safe position
    % (nothing to retreat from, it does not move) is shown as 11 = wait at safe position
    atSafe = norm(env.tool - prm.geo.poseSolo) < 0.03 && env.measuredSpeed < 0.01;
    aPlot = aF; aNPlot = out.aN;
    if atSafe && aF==9, aPlot = 11; mot = 0; end
    if atSafe && out.aN==9, aNPlot = 11; end
    actionFileRow(actionFile, k, tNow, tc.mode, prevSt.sessionOn, out, aF, aPlot, act);
    r = struct('t',tNow,'Fp',O(1),'Fm',O(2),'A',O(3), ...
        'bFp',squeeze(sum(sum(Bm,1),2))'*[0;1;2],'bFm',squeeze(sum(sum(Bm,1),3))*[0;1;2], ...
        'bA',squeeze(sum(sum(Bm,2),3))'*[0;1;2],'handDist',O(14),'clearance',O(15),'Z',O(9),'R',O(6), ...
        'verified',double(geo.verified),'aN',aNPlot,'aF',aPlot,'motion',mot,'level',prevSt.level, ...
        'dialogCode',find(strcmp(dialogs,prevSt.dialog))-1,'button',btn,'sessionOn',double(prevSt.sessionOn), ...
        'inAssembly',double(geo.inAssemblyArea),'mode',tc.mode,'robotSpeed',robotSpeedNow(geo, env), ...
        'mentalScore',geo.sig.mentalScore,'physicalF',geo.sig.physicalF,'awayS',geo.sig.awayS, ...
        'cmdSpeed',cmdSpeed,'jointSpeed',geo.measuredJointSpeed,'conf',O(7),'cause',causeCode);
    lp = livePlotHRC(lp, prm, r);
    % ---------------- scenario recording: G starts, X ends and makes the figure
    if btn==4 && isempty(rec)
        rec = struct('t0',tNow,'start',datetime('now'),'mode0',tc.mode,'data',zeros(0,numel(recCols)), ...
            'participant',participant,'condition',condition);
        rec.reason = {}; rec.adapt = {}; rec.phase = {};   % per epoch: supervisor reason, adaptation, task phase
        fprintf('scenario recording started (%s mode)\n', modeName(tc.mode));
    end
    if ~isempty(rec)
        rec.data(end+1,:) = [tNow-rec.t0, O(1), O(2), O(3), O(14), O(15), aPlot, aNPlot, mot, tc.mode, ...
            r.dialogCode, double(geo.verified), tl.requests, tl.deliveries, btn, O(9), O(6), ...
            robotSpeedNow(geo, env), tNow-rd.lastRx, rd.count, double(geo.armEstimated), double(geo.inAssemblyArea), ...
            O(7), prevSt.level, aF, out.aN, robotScale, double(geo.realRobot), ...
            geo.sig.mentalScore, geo.sig.physicalF, geo.sig.awayS, cmdSpeed, geo.measuredJointSpeed, causeCode];
        rec.reason{end+1,1} = out.supervisor.reason;
        rec.adapt{end+1,1} = out.adaptReason;
        rec.phase{end+1,1} = tl.phase;
        if btn==5
            rec = finishScenario(rec, recCols, prm, figWidth, figHeight);
        end
    end
    logRows(end+1,:) = {tNow, O(1), O(2), O(3), O(4), O(7), O(8), geo.arm, geo.inAssemblyArea, O(14), O(15), O(9), O(6), ...
        geo.verified, tc.mode, tl.phase, tc.instruction, act2name(out.aN, act), act2name(out.aA, act), ...
        act2name(aF, act), out.supervisor.reason, out.adaptReason, prevSt.level, prevSt.dialog, ...
        prevSt.message, prevSt.sessionOn, estop, keyTxt, tNow - rd.lastRx, geo.realRobot, geo.headDistance, robotCmd, ...
        out.speedScale, out.speedReason, causeLabel}; %#ok<SAGROW>
    if mod(k, 120)==0, writeLog(logFile, logRows); end   % every minute
    % ---------------- experiment video: G / R start, P / X stop
    if recordVideo
        if isempty(vid) && prevSt.sessionOn && (btn==4 || btn==1)
            vidPart = vidPart + 1;
            vid = experimentVideoHRC('start', videoRoot, vidPart);
        end
        if ~isempty(vid)
            if oneWindow, figs = {h.fig}; else, figs = {h.fig, lp.fig}; end
            vrow = struct('t_wall', 0, 't_s', tNow, 'session', double(prevSt.sessionOn), 'mode', modeName(tc.mode), ...
                'phase', tl.phase, 'final', aPlot, 'final_name', act2name(aF, act), 'planner', act2name(out.aN, act), ...
                'robot_action', robotCmd, 'Fm', O(2), 'Fp', O(1), 'attention', O(3), 'level', prevSt.level, ...
                'dialog', prevSt.dialog, 'zone', O(9), 'risk', O(6), 'hand_m', O(14), 'clearance_m', O(15), ...
                'head_m', geo.headDistance, 'verified', double(geo.verified), 'supervisor', out.supervisor.reason, ...
                'message', prevSt.message);
            vrow.t_wall = posixtime(datetime('now','TimeZone','local'));
            vid = experimentVideoHRC('frame', vid, figs, vrow);
            if btn==2 || btn==5
                vid = experimentVideoHRC('stop', vid, videoComposer);
            end
        end
    end
    pause(max(prm.dt - toc(loopStart), 0.001));
end
catch err                                        % save the log and free the port, then show the error
    if commandRealRobot, writeRobotAction(actionsFile, 10, robotCmd, NaN, 'liveHRC error: STOP'); end
    if ~isempty(pw) && ~isempty(pw.fig) && isvalid(pw.fig), delete(pw.fig); end
    if ~isempty(vid), experimentVideoHRC('stop', vid, videoComposer); end
    writeLog(logFile, logRows);
    if ~isempty(rec), finishScenario(rec, recCols, prm, figWidth, figHeight); end
    clear rd
    rethrow(err);
end
if commandRealRobot, writeRobotAction(actionsFile, 10, robotCmd, NaN, 'liveHRC ended: STOP'); end
if ~isempty(pw) && ~isempty(pw.fig) && isvalid(pw.fig), delete(pw.fig); end
if ~isempty(vid), experimentVideoHRC('stop', vid, videoComposer); end
writeLog(logFile, logRows);
if ~isempty(rec), finishScenario(rec, recCols, prm, figWidth, figHeight); end   % Esc during a scenario
clear rd
fprintf('stopped; log saved to %s\n', logFile);

function rec = finishScenario(rec, recCols, prm, figWidth, figHeight)
% save the recorded scenario and its report (scenarioReportHRC): folder
% results\scenario_<mode>_<date>\ with the figure (.pdf/.png/.fig), the data
% (.mat and _data.csv), the metrics (_metrics.csv) and the events (_events.csv)
rec.cols = cell2struct(num2cell(1:numel(recCols)), recCols, 2);
modes = unique(rec.data(:, rec.cols.mode));
if isscalar(modes), m = modeName(modes); else, m = 'mixed'; end
rec.name = [m '_' char(rec.start,'yyyyMMdd_HHmmss')];
if isfield(rec,'participant'), rec.name = [rec.participant '_' rec.condition '_' rec.name]; end
scenarioReportHRC(rec, prm, struct('width',figWidth,'height',figHeight));
rec = [];
end

function [cmd, scale] = writeRobotAction(file, n, prev, t, why, scale, prevScale)
% actions.md <- n and the speed factor (line 2 'speed_scale=..', 1 = file speed; solo fatigue / distance)
% only when one of them changes (factor: by 0.05 or more, or back to 1); temp file + move, so robot_code.py
% never reads half a file
if nargin < 6, scale = 1; end
if nargin < 7, prevScale = 1; end
scale = round(min(max(scale, 0), 1) * 100) / 100;
cmd = prev;
if n == prev && abs(scale - prevScale) < 0.05 && ~(scale == 1 && prevScale ~= 1)
    scale = prevScale;
    return
end
names = {'to storage','solo normal','solo slow','approach handling point','hold at handling point', ...
    'collaborative normal','collaborative slow','collaborative hold','stop','retreat to safe', ...
    'PROTECTIVE STOP','wait at safe position'};
tmp = [file '.tmp'];
fid = fopen(tmp, 'w');
if fid < 0
    warning('liveHRC:actions', 'cannot write %s', tmp);
    scale = prevScale;
    return
end
fprintf(fid, '%d\nspeed_scale=%.2f\n', n, scale);
fclose(fid);
movefile(tmp, file, 'f');
if n == prev
    fprintf('[%6.1f s] ROBOT speed x%.2f (action %d)  [%s]\n', t, scale, n, why);
elseif scale < 1
    fprintf('[%6.1f s] ROBOT -> action %d (%s) speed x%.2f  [%s]\n', t, n, names{n+1}, scale, why);
else
    fprintf('[%6.1f s] ROBOT -> action %d (%s)  [%s]\n', t, n, names{n+1}, why);
end
cmd = n;
end

function s = modeName(m)
names = {'solo','assistive','collaborative'};
s = names{m+1};
end

function s = act2name(a, act)
if a==0, s = 'tr'; else, s = act.short{a}; end
end

function actionFileInit(file, act)
% action.md: header with the code list, then the table of robot commands
fid = fopen(file, 'w');
if fid < 0, warning('cannot write %s', file); return; end
fprintf(fid, '# Robot commands - live HRC session\n\n');
fprintf(fid, 'Started: %s  \n', char(datetime('now','Format','yyyy-MM-dd HH:mm:ss')));
fprintf(fid, 'One row per decision epoch (0.5 s). The robot executes the **code** of the LAST row.\n\n');
fprintf(fid, '## Codes\n\n| code | short | action | speed |\n|---|---|---|---|\n');
fprintf(fid, '| 0 | tr | task-layer motion to the target pose (tool fetch, go to / come back from the safe position) | %.1f |\n', speedOf(0, act));
for a = 1:10
    fprintf(fid, '| %d | %s | %s | %.1f |\n', a, act.short{a}, act.names{a}, speedOf(a, act));
end
fprintf(fid, '| 11 | wt | wait at safe position (retreat 9 while already at the safe position) | 0.0 |\n\n');
fprintf(fid, 'speed: fraction of normal robot speed (1 normal, 0.3 slow, 0 stand still).  \n');
fprintf(fid, 'source: planner = POMDP decision executed as it is; fatigue = changed by fatigueAdaptationHRC; ');
fprintf(fid, 'safety = changed by safetySupervisorHRC; task = task-layer motion (code 0); session = session stopped.\n\n');
fprintf(fid, '## Commands\n\n');
fprintf(fid, '| step | t_s | mode | session | planner | adapted | supervisor | code | short | speed | source | target_xyz_m | reason |\n');
fprintf(fid, '|---|---|---|---|---|---|---|---|---|---|---|---|---|\n');
fclose(fid);
end

function actionFileRow(file, k, t, M, sessionOn, out, aF, aOut, act)
% one command row; aOut = executed code (11 = retreat while already at the safe position)
modes = {'solo','assistive','collaborative'};
if aOut==11, short = 'wt'; elseif aOut==0, short = 'tr'; else, short = act.short{aOut}; end
reason = '';
if ~sessionOn
    src = 'session'; reason = 'session_stopped_go_safe';
elseif aF==0 && out.status.robotGoSafe
    src = 'task'; reason = 'rest_go_safe';
elseif aF==0
    src = 'task'; reason = 'transfer';
elseif aF ~= out.aA
    src = 'safety'; reason = out.supervisor.reason;
elseif out.aA ~= out.aN
    src = 'fatigue'; reason = out.adaptReason;
else
    src = 'planner';
    if isfield(out.supervisor,'reason'), reason = out.supervisor.reason; end
end
if aF==0 && ~isempty(out.target)
    tgt = sprintf('%.3f %.3f %.3f', out.target(1), out.target(2), out.target(3));
else
    tgt = '-';
end
spd = speedOf(aOut, act);
fid = fopen(file, 'a');
if fid < 0, return; end
fprintf(fid, '| %d | %.2f | %s | %d | %s | %s | %s | %d | %s | %.1f | %s | %s | %s |\n', k, t, modes{M+1}, ...
    double(sessionOn), codeName(out.aN, act), codeName(out.aA, act), codeName(aF, act), aOut, short, spd, ...
    src, tgt, strrep(reason, '|', '/'));
fclose(fid);
end

function s = codeName(a, act)
if a==0, s = 'tr'; else, s = act.short{a}; end
end

function v = speedOf(a, act)
% fraction of normal speed: act.motion 2 normal, 1 slow (0.3), 0 stopped; tr moves slowly
if a==0
    v = 0.3;
elseif a==10 || a==11
    v = 0;
else
    lv = [0 0.3 1];
    v = lv(act.motion(a)+1);
end
end

function writeLog(file, rows)
if isempty(rows), return; end
T = cell2table(rows, 'VariableNames', {'time_s','Fp','Fm','attention','present','confidence','face', ...
    'arm','handInAssemblyArea','handDist_m','clearance_m','zone','risk','geometryVerified','mode','phase','instruction', ...
    'planner','adapted','final','supervisor','adaptation','fatigueLevel','dialogue','message', ...
    'sessionOn','estop','keys','cameraAge_s','realRobot','headDist_m','robotAction','speedScale','speedAdaptation','speedCause'});
writetable(T, file);
end

function v = robotSpeedNow(geo, env)
% measured tool speed [m/s]: the real robot (camera program) when its pose is received, else the virtual robot
if isfield(geo,'realRobot') && geo.realRobot && isfield(geo,'measuredRobotSpeed') && isfinite(geo.measuredRobotSpeed)
    v = geo.measuredRobotSpeed;
else
    v = env.measuredSpeed;
end
end
