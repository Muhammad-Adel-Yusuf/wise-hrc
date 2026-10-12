function [aF, out, mem] = hrcStepHRC(O, geo, ev, tc, mem, tab)
% ONE DECISION CYCLE OF THE FRAMEWORK (call it every prm.dt = 0.5 s).
% This is the single entry point for a real-time session and for the tests:
%
%   task layer --tc-->  [ 2 DECISION LAYER                         ]  --> 3 SAFETY SUPERVISOR --> aF
%                       [ belief -> QMDP planner -> fatigue monitor ]
%                       [        -> fatigue adaptation              ]
%   task layer <--out.status-- (fatigue level, rest warning, rest, help useful)
%
% O    : observation vector (syntheticObservationHRC / realCameraObservationAdapter)
% geo  : observed safety geometry (geometryObservationHRC)
% ev   : behavioural events of the previous epoch [backtrack view hesitation correction]
% tc   : task command (taskCommandHRC), from the task layer
% mem  : framework memory ([] at the start, or struct('mon',struct('par',s))
%        to change some monitor settings of prm.fatigue for this run)
% tab  : struct with the planner tables P, V, N, T, R, MK
%
% WORKER BUTTONS (O(18)): 1 RESUME, 2 PAUSE (see humanStateMonitorHRC),
%   4 START session: the planner starts (belief and fatigue monitor start fresh)
%   5 STOP session : the planner stops; the robot goes to tc.safePose and waits
%   The session starts automatically when prm.session.autoStart = true (tests),
%   otherwise only after START (real sessions). STOP is not an emergency stop:
%   the emergency stop stays a separate hardware signal (O(10)).
% REST PREDICTION: while the worker rests, the belief predicts him like an
%   absent worker (faster recovery, TransProbHRC); observations still correct it.
%
% aF   : final action for the robot (0 = task-layer motion to out.target)
% out  : aN (planner), aA (after fatigue adaptation), aF (after the safety
%        supervisor), target (pose for a = 0), status (humanStateMonitorHRC),
%        adaptReason, supervisor (info of safetySupervisorHRC), b (belief)
prm = stateDefinitionHRC();
act = actionDefinitionHRC();
if isempty(mem) || ~isfield(mem,'k')
    mon0 = [];                                   % mem = struct('mon',struct('par',...)) passes
    if isstruct(mem) && isfield(mem,'mon')       % monitor settings (humanStateMonitorHRC)
        mon0 = mem.mon;
    end
    autoStart = prm.session.autoStart;
    if isstruct(mem) && isfield(mem,'autoStart'), autoStart = mem.autoStart; end
    mem = struct('b',[],'aPrev',2,'Sop',[],'sup',[],'V',0,'k',0,'resting',false, ...
        'sessionOn',autoStart,'solo',[]);
    mem.mon = mon0;
end
mem.k = mem.k + 1;
% ---------------- task-layer signals enter the observation
O(16) = tc.handoverActive;
O(17) = tc.toolTaken;
% ---------------- session buttons
cmd = 0;
if numel(O) >= 18 && ~isnan(O(18)), cmd = O(18); end
if cmd==4 && ~mem.sessionOn
    mem.sessionOn = true;                        % fresh start of belief and monitor
    mem.b = []; mem.resting = false;
    if isstruct(mem.mon) && isfield(mem.mon,'par'), mem.mon = struct('par',mem.mon.par); else, mem.mon = []; end
elseif cmd==5
    mem.sessionOn = false;
end
Om = O;                                          % the monitor sees RESUME / PAUSE only
if any(cmd==[4 5]) || ~mem.sessionOn, Om(18) = 0; end
% ---------------- observed context and belief
M = tc.mode; I = tc.instruction;
H = O(4); if isnan(H), H = 1; end
Zc = O(9); if isnan(Zc), Zc = 0; end
Rc = O(6); if isnan(Rc), Rc = 2; end
So = [M I H 0 0 0 Zc mem.V Rc];
if isempty(mem.b)
    [b,il] = BeliefStateHRC([],[],[],So,O,[],[]);
else
    Sp = mem.Sop;
    if mem.resting, Sp(3) = 0; end               % resting worker: predicted like an absent worker
    [b,il] = BeliefStateHRC(mem.b,mem.aPrev,Sp,So,O,ev,0.5);
end
% ---------------- fatigue monitor and worker dialogue
[status, mem.mon] = humanStateMonitorHRC(b, Om, ev, tc, mem.mon);
if ~mem.sessionOn                                % session stopped: robot idle at its safe position
    status.dialog = 'none'; status.restActive = false; status.restRecommended = false;
    status.holdForWarning = false; status.robotGoSafe = true;
    status.message = 'Session stopped. Press START to begin.';
end
status.sessionOn = mem.sessionOn;
mem.resting = status.restActive;
% ---------------- planner (or task-layer motion)
% Reallocation (8) without tc.allowReallocation: in solo and collaborative mode
% the planner may not propose it, so it chooses among the actions that will
% really be executed (collaborative: cn / cs / ch, solo: sc / ss) instead of
% proposing 8 and having it replaced afterwards. In assistive mode 8 is the
% robot's assistance itself and stays allowed.
taskMask = tc.allowedActions;
if ~tc.allowReallocation && M~=1
    if isempty(taskMask), taskMask = ones(1,10); end
    taskMask(8) = 0;
end
if ~mem.sessionOn
    aN = 0;
elseif ~isempty(tc.transferTarget)
    aN = 0;
elseif isempty(taskMask)
    aN = pomdpActionHRC(b,il,tab.P,tab.V,tab.N,tab.T,tab.R,tab.MK);
else
    aN = pomdpActionHRC(b,il,tab.P,tab.V,tab.N,tab.T,tab.R,tab.MK,taskMask);
end
% ---------------- fatigue adaptation
[aA, adaptReason] = fatigueAdaptationHRC(aN, tc, status);
if status.robotGoSafe
    target = tc.safePose;
else
    target = tc.transferTarget;
end
% ---------------- safety supervisor (unchanged, never reads the belief)
[aF, mem.sup, info] = safetySupervisorHRC(aA, O, geo, M, mem.sup);
% ---------------- solo speed adaptation to fatigue and distance (only slower / stop)
if ~isfield(mem,'solo'), mem.solo = []; end
[aF, speedScale, speedReason, mem.solo] = soloFatigueSpeedHRC(aF, M, geo, status.level, mem.solo, prm, O(3));
% ---------------- memory and output
mem.b = b; mem.aPrev = aF; mem.Sop = So;
if aF==0, mem.V = 1; else, mem.V = act.motion(aF); end
out.aN = aN; out.aA = aA; out.aF = aF; out.target = target;
out.status = status; out.adaptReason = adaptReason; out.supervisor = info; out.b = b;
out.speedScale = speedScale; out.speedReason = speedReason;   % solo fatigue / distance speed factor
out.dt = prm.dt;
end
