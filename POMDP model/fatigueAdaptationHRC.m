function [aA, reason] = fatigueAdaptationHRC(aN, tc, status)
% FATIGUE ADAPTATION (end of the decision layer, before the safety supervisor).
% Adapts the planner action to the sustained fatigue level and to the
% worker dialogue (humanStateMonitorHRC). It can only make the robot slower,
% make it wait, or send it to its safe position; it never speeds it up.
% The safety supervisor checks the result exactly as before.
%
% aN     : planner action (1..10) or 0 = task-layer transfer
% tc     : task command (taskCommandHRC)
% status : monitor status
% aA     : adapted action, reason : '' when unchanged
%
% Rules (in this order):
%   rest, assistive / collaborative -> 0, the robot goes to tc.safePose and waits
%   rest, solo                      -> unchanged (the robot keeps working)
%   warning, collaborative          -> 7 (stopped hold) until the worker answers
%   warning, solo / assistive       -> unchanged (the robot continues, message only)
%   collaborative, level >= medium  -> normal speed 5 becomes slow and smooth 6
%   no reallocation allowed (tc.allowReallocation = 0):
%       collaborative 8 (robot takes over) -> 6 (keep working together, slowly)
%       solo 8 (reallocation)              -> 1 (the safety supervisor still
%                                            slows it when separation is not verified)
%   (in assistive mode 8 is the robot's assistance itself and is kept)
M = tc.mode;
aA = aN; reason = '';
if status.robotGoSafe
    aA = 0; reason = 'rest_robot_to_safe_position';
    return
end
if status.holdForWarning
    aA = 7;
    if aN ~= 7, reason = 'fatigue_warning_hold'; end
    return
end
if aA==8 && ~tc.allowReallocation && M~=1
    if M==2
        aA = 6;
    else
        aA = 1;
    end
    reason = 'reallocation_not_enabled';
end
if M==2 && status.level >= 1 && aA==5
    aA = 6; reason = 'fatigue_slow_collaborative';
end
end
