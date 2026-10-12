function n = robotActionHRC(aF, phase, mode, robotGoSafe, tool, g)
% Number of the REAL robot's action file (tasks/task<n>.txt, executed by
% robot_code/robot_code.py through actions.md) for one decision epoch.
%
% aF          final action of hrcStepHRC (0 = task-layer motion, 1..10 = POMDP actions)
% phase       task-layer phase (taskLayerLiveHRC: MOVE, WORK, IDLE, FETCH, GRIP, APPROACH, HOLD, RETURN)
% mode        0 solo, 1 assistive, 2 collaborative
% robotGoSafe true while the robot is sent to its safe position (rest, session stopped)
% tool        robot tool position in W [m] (the real robot's when it is commanded)
% g           prm.geo (places)
%
% Action files (written by the user, see tasks/task*.txt):
%   0 to the storage   1 solo normal   2 solo slow   3 approach the handling point   4 hold at the handling point
%   5 collaborative normal   6 collaborative slow   7 collaborative hold   8 stop   9 retreat to the safe position
%   10 protective stop   11 stop and wait at the safe position
% Mapping:
%   protective stop (10)                         -> 10
%   task-layer motion (aF = 0):  to the safe position (rest, session stopped, RETURN, MOVE in solo / assistive)
%                                -> 9, or 11 once the robot is there;  FETCH / GRIP -> 0;  APPROACH -> 3;
%                                MOVE in collaborative mode (to the collaborative work point) -> 5 (speed scaled by
%                                fatigue in liveHRC);  else -> 7 (hold)
%   planner actions 1..9         -> the same number; retreat (9) at the safe position -> 11
% 'at the safe position' only within 8 mm: inside the 1 cm the task layer needs to start the work (MOVE -> WORK /
% IDLE). With 3 cm the retreat was stopped (11) ~2 cm short and the task layer waited forever (2026-10-09).
atSafe = norm(tool(:)' - g.poseSolo(:)') < 0.008;
if aF == 10
    n = 10;
elseif aF == 0
    if robotGoSafe || any(strcmp(phase, {'RETURN','IDLE'})) || (strcmp(phase,'MOVE') && mode ~= 2)
        n = 9;
    elseif any(strcmp(phase, {'FETCH','GRIP'}))
        n = 0;
    elseif strcmp(phase, 'APPROACH')
        n = 3;
    elseif strcmp(phase, 'MOVE') && mode == 2
        n = 5;                                   % approach the collaborative work point at normal speed;
                                                 % liveHRC scales it by the fatigue level (prm.approachSpeedScale)
    else
        n = 7;                                   % unknown task-layer motion: hold, never guess a path
    end
else
    n = aF;
end
if n == 9 && atSafe
    n = 11;
end
end
