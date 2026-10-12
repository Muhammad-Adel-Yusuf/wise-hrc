function [status, mon] = humanStateMonitorHRC(b, O, ev, tc, mon)
% FATIGUE MONITOR AND WORKER DIALOGUE (decision layer).
% Turns the belief into a sustained fatigue level, decides when to warn the
% worker, and follows his answer. It never moves the robot itself: it gives
% directives to fatigueAdaptationHRC and recommendations to the task layer.
%
% b   : belief over [Fp Fm A] (27x1, BeliefStateHRC)
% O   : observation; O(3) attention, O(4) presence, O(18) worker button
%       (0/NaN none, 1 RESUME = continue / back to work, 2 PAUSE = rest now,
%        3 = same as RESUME; START / STOP of the session are handled by hrcStepHRC)
% ev  : behavioural events [backtrack prolongedView hesitation correction]
% tc  : task command (taskCommandHRC)
% mon : monitor memory ([] at the start; struct('par',s) to override some
%       prm.fatigue values, e.g. struct('par',struct('snoozeContinue',60)))
%
% FATIGUE LEVEL (0 low, 1 medium, 2 high), from the belief:
%   "high now"   when P(Fp = high) or P(Fm = high) > prm.fatigue.highProb
%   "medium now" when P(Fp >= medium) or P(Fm >= medium) > prm.fatigue.medProb
%   the level goes up only after "high now" lasted highHold seconds (medium:
%   medHold) and goes down only after a lower value lasted downHold seconds,
%   so one wrong reading does not change anything.
% DIALOGUE (status.dialog):
%   'none'      no warning
%   'warning'   level high: the worker is asked to rest
%               collaborative -> the robot stops and waits for the answer
%                                (no answer = it keeps waiting)
%               solo / assistive -> the robot continues, the message stays
%   'continued' the worker pressed RESUME (continue): no new warning for
%               snoozeContinue seconds or until the level drops
%   'rest'      the worker pressed PAUSE, after a warning or at any time;
%               solo: the robot keeps working; assistive / collaborative: the
%               robot goes to its safe position and waits. Ends when the
%               worker presses RESUME while he is at the workstation
%               (presence O(4) not 0; a RESUME pressed while away is ignored),
%               or after maxRest.
%   No warning is started while the robot holds a tool at the handover point.
% HELP WOULD BE USEFUL (status.helpUseful, recommendation only, the robot
%   does not act on it): level >= medium AND (struggle events in the last
%   struggleWindow seconds >= struggleEvents OR level high), not during a
%   handover and not while the worker is distracted.
prm = stateDefinitionHRC();
dt = prm.dt;
if isempty(mon) || ~isfield(mon,'level')
    par = prm.fatigue;                           % mon = struct('par',...) overrides some values
    if isstruct(mon) && isfield(mon,'par')
        f = fieldnames(mon.par);
        for i = 1:numel(f), par.(f{i}) = mon.par.(f{i}); end
    end
    mon = struct('level',0,'c1',0,'c2',0,'cb',0,'dialog','none','snooze',-Inf, ...
        'restStart',NaN,'t',0,'hist',zeros(1,0),'par',par);
end
p = mon.par;
if nargin < 3 || isempty(ev), ev = zeros(1,4); end
t = mon.t;
% ---------------- belief -> fatigue level
B = reshape(b,3,3,3);                            % (A, Fm, Fp)
pFp = squeeze(sum(sum(B,1),2));                  % 3x1, P(Fp = 0,1,2)
pFm = squeeze(sum(sum(B,1),3))';                 % 3x1, P(Fm = 0,1,2)
hiP = pFp(3); hiM = pFm(3);
meP = pFp(2) + pFp(3); meM = pFm(2) + pFm(3);
if max(hiP,hiM) > p.highProb
    lvlNow = 2;
elseif max(meP,meM) > p.medProb
    lvlNow = 1;
else
    lvlNow = 0;
end
if lvlNow >= 1, mon.c1 = mon.c1 + 1; else, mon.c1 = 0; end
if lvlNow >= 2, mon.c2 = mon.c2 + 1; else, mon.c2 = 0; end
if mon.level < 2 && mon.c2*dt >= p.highHold
    mon.level = 2; mon.cb = 0;
elseif mon.level < 1 && mon.c1*dt >= p.medHold
    mon.level = 1; mon.cb = 0;
end
if lvlNow < mon.level, mon.cb = mon.cb + 1; else, mon.cb = 0; end
if mon.cb*dt >= p.downHold
    mon.level = lvlNow; mon.cb = 0;
end
% ---------------- struggle events in the last window
nW = round(p.struggleWindow/dt);
mon.hist = [mon.hist sum(ev(2:4))];
if numel(mon.hist) > nW, mon.hist = mon.hist(end-nW+1:end); end
% ---------------- dialogue with the worker
resp = 0;
if numel(O) >= 18 && ~isnan(O(18)), resp = O(18); end
if resp==3, resp = 1; end                        % "rest finished" = RESUME
present = isnan(O(4)) || O(4)==1;
ignored = false;
M = tc.mode;
hov = tc.handoverActive==1;
dg = mon.dialog;
switch dg
    case {'none','continued'}
        if resp==2                               % the worker asks for a rest himself
            dg = 'rest'; mon.restStart = t;
        elseif strcmp(dg,'none')
            if mon.level==2 && t >= mon.snooze && ~hov
                dg = 'warning';
            end
        elseif t >= mon.snooze || mon.level < 2
            dg = 'none';
        end
    case 'warning'
        if resp==1
            dg = 'continued'; mon.snooze = t + p.snoozeContinue;
        elseif resp==2
            dg = 'rest'; mon.restStart = t;
        elseif mon.level < 2
            dg = 'none';                         % fatigue went down without an answer
        end
    case 'rest'
        if (resp==1 && present) || t - mon.restStart >= p.maxRest
            dg = 'none'; mon.snooze = t + p.snoozeRest;
        elseif resp==1
            ignored = true;                      % RESUME pressed while away from the workstation
        end
end
mon.dialog = dg;
mon.t = t + dt;
% ---------------- status report
distracted = ~isnan(O(3)) && O(3)==2;
status.pHighPhysical = hiP;
status.pHighMental = hiM;
status.levelNow = lvlNow;
status.level = mon.level;
if hiP > p.highProb && hiM > p.highProb
    status.fatigueType = 'physical+mental';
elseif hiP >= hiM
    status.fatigueType = 'physical';
else
    status.fatigueType = 'mental';
end
status.dialog = dg;
status.response = resp;
status.resumeIgnored = ignored;
status.restRecommended = strcmp(dg,'warning');
status.restActive = strcmp(dg,'rest');
status.holdForWarning = strcmp(dg,'warning') && M==2;
status.robotGoSafe = strcmp(dg,'rest') && any(M==[1 2]);
status.struggleEvents = sum(mon.hist);
status.helpUseful = mon.level >= 1 && (sum(mon.hist) >= p.struggleEvents || mon.level==2) ...
    && ~hov && ~distracted;
switch dg
    case 'warning'
        if M==2
            status.message = 'You seem tired. The robot has stopped. Press ENTER (RESUME) to continue or P to rest.';
        else
            status.message = 'You seem tired. Please consider a rest (press P). ENTER = continue. The robot continues its work.';
        end
    case 'rest'
        if ignored
            status.message = 'Please come back to the workstation, then press ENTER.';
        else
            status.message = 'Rest. Press ENTER when you are back at the workstation.';
        end
    otherwise
        status.message = '';
end
status.taskType = tc.taskType;
end
