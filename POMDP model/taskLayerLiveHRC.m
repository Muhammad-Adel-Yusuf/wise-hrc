function [tc, tl] = taskLayerLiveHRC(tl, env, prevStatus, cmd, handToTool, latched, prm)
% DEFAULT TASK LAYER of the live tests (real worker, virtual robot).
% It only says WHAT the robot works on (task command, taskCommandHRC); HOW is
% decided by hrcStepHRC. A future task-allocation layer replaces this function.
%
% [~, tl] = taskLayerLiveHRC(mode)  start in mode 0 solo, 1 assistive, 2 collaborative
% [tc, tl] = taskLayerLiveHRC(tl, env, prevStatus, cmd, handToTool, latched, prm)
%   env        : virtual robot (tool position env.tool)
%   prevStatus : status of the previous decision epoch (hrcStepHRC), [] at the start
%   cmd        : keys of this epoch: cmd.mode (-1 or 0/1/2), cmd.request (tool
%                wanted), cmd.taken (worker confirms he has the tool)
%   handToTool : measured hand-to-tool distance [m] (NaN if unknown)
%   latched    : 1 while the safety supervisor holds a stop / handover hold
%
% Phases
%   MOVE      task-layer motion to the work pose of the mode (after a mode change,
%             a rest or START); solo / assistive: safe position, collaborative:
%             collaborative work point
%   WORK      solo or collaborative work, decided by the planner (solo: only
%             solo normal / solo slow / protective stop, at the safe position)
%   assistive "bring me a tool": IDLE -> (key T) FETCH -> GRIP -> APPROACH -> HOLD
%             -> (hand at the tool for 1 s, or key K) RETURN -> IDLE
%             APPROACH and RETURN are task-layer motions to the calibrated handover
%             point / safe position (slow, monitored by the safety supervisor:
%             it still stops the robot for hazards, distraction near the robot,
%             unverified geometry). They are not left to the planner because with
%             real places the worker's hand can sit on the border between the
%             collaborative and assistive distances, and the planner would then
%             alternate approach / retreat. The planner decides the hold (HOLD).
% While the robot is sent to its safe position (rest or session stopped) the
% phases are frozen; afterwards the robot moves back (an unfinished handover
% starts again with the approach).
if isempty(tl) || isnumeric(tl)
    mode = 1;
    if isnumeric(tl) && ~isempty(tl), mode = tl; end
    tl = struct('mode',mode,'phase','MOVE','I',1,'gripCount',0,'toolTaken',0, ...
        'graspCount',0,'requests',0,'deliveries',0,'wasPaused',false,'pendingRequest',false);
    tc = [];
    return
end
g = prm.geo;
paused = ~isempty(prevStatus) && prevStatus.robotGoSafe;
work = workPose(tl.mode, g);
% ---------------- keys
if cmd.mode >= 0 && cmd.mode ~= tl.mode
    tl.mode = cmd.mode; tl.phase = 'MOVE'; tl.I = 1; tl.toolTaken = 0; tl.pendingRequest = false;
    tl.gripCount = 0; tl.graspCount = 0;
    work = workPose(tl.mode, g);
end
if ~isfield(tl,'pendingRequest'), tl.pendingRequest = false; end
if cmd.request && tl.mode==1 && ~strcmp(tl.phase,'IDLE') && ~any(strcmp(tl.phase,{'FETCH','GRIP','APPROACH','HOLD'}))
    tl.pendingRequest = true;                    % T before the robot waits at its safe position: served when it does
end
if (cmd.request || tl.pendingRequest) && tl.mode==1 && strcmp(tl.phase,'IDLE')
    tl.pendingRequest = false;
    tl.phase = 'FETCH'; tl.I = 2; tl.requests = tl.requests + 1;
end
% ---------------- phases (frozen while the robot is at / going to its safe position)
if ~paused
    if tl.wasPaused
        if any(strcmp(tl.phase,{'WORK','IDLE','MOVE'}))
            tl.phase = 'MOVE';                       % back to the work pose
        elseif strcmp(tl.phase,'GRIP')
            tl.phase = 'FETCH';                      % back to the storage, grip again
        elseif any(strcmp(tl.phase,{'APPROACH','HOLD'})) && tl.toolTaken
            tl.phase = 'RETURN'; tl.I = 3;           % tool already handed over
        elseif any(strcmp(tl.phase,{'APPROACH','HOLD'}))
            tl.phase = 'APPROACH';                   % handover starts again
        end
    end
    switch tl.phase
        case 'MOVE'
            if norm(env.tool - work) < 0.01
                if tl.mode==1, tl.phase = 'IDLE'; else, tl.phase = 'WORK'; end
            end
        case 'FETCH'
            if norm(env.tool - g.poseStorage) < 0.01, tl.phase = 'GRIP'; tl.gripCount = 0; end
        case 'GRIP'
            tl.gripCount = tl.gripCount + 1;
            if tl.gripCount > 4, tl.phase = 'APPROACH'; end
        case 'APPROACH'
            if norm(env.tool - g.poseAssist) < 0.03, tl.phase = 'HOLD'; tl.graspCount = 0; end
        case 'HOLD'
            if norm(env.tool - g.poseAssist) >= 0.03
                tl.phase = 'APPROACH';
            elseif ~tl.toolTaken
                if handToTool <= g.graspDistance + 0.05
                    tl.graspCount = tl.graspCount + 1;
                else
                    tl.graspCount = 0;
                end
                if tl.graspCount >= 2 || cmd.taken
                    tl.toolTaken = 1; tl.deliveries = tl.deliveries + 1;
                end
            elseif ~latched
                tl.phase = 'RETURN'; tl.I = 3;           % tool taken and hand clear
            end
        case 'RETURN'
            if norm(env.tool - g.poseSolo) < 0.01
                tl.phase = 'IDLE'; tl.I = 1; tl.toolTaken = 0;
            end
    end
end
tl.wasPaused = paused;
% ---------------- task command
tc = taskCommandHRC(tl.mode, tl.I);
tc.safePose = g.poseSolo;
tc.toolTaken = tl.toolTaken;
switch tl.phase
    case 'MOVE'
        tc.transferTarget = work;
    case 'FETCH'
        tc.transferTarget = g.poseStorage;
    case 'GRIP'
        tc.transferTarget = env.tool;            % stay at the storage while gripping
    case 'APPROACH'
        tc.transferTarget = g.poseAssist;        % carry the tool to the handover point
    case 'RETURN'
        tc.transferTarget = g.poseSolo;          % back to the safe position
    case 'IDLE'
        % assistive, nothing requested: the robot waits at its safe position. Only
        % "stay away" (retreat, shown as "wait at safe position") or a protective
        % stop: the robot never approaches the worker without a request (key T).
        tc.allowedActions = zeros(1,10);
        tc.allowedActions([9 10]) = 1;
    case 'WORK'
        if tl.mode==0
            % solo: the robot works on its own task at its safe position. The planner
            % chooses only how (normal / slow / protective stop); "retreat" is not
            % offered because the robot already is at its separated place and would
            % just stand still. The safety supervisor still slows or stops it.
            tc.allowedActions = zeros(1,10);
            tc.allowedActions([1 2 10]) = 1;
        end
    case 'HOLD'
        if ~paused
            tc.handoverActive = 1;
            tc.allowedActions = zeros(1,10);
            tc.allowedActions([4 10]) = 1;
        end
end
end

function p = workPose(mode, g)
if mode==2
    p = g.poseCollab;
else
    p = g.poseSolo;
end
end
