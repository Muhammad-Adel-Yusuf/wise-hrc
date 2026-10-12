function [code, label] = speedCauseHRC(aF, aN, out, phase, mode, apScale, supCause, conf)
% CAUSE of a robot speed reduction in one decision epoch (for the plots, the log
% and the analysis), read from the decision layers in their order of authority:
% safety supervisor > fatigue adaptation > solo fatigue / distance rule >
% approach factor > planner. 0 when the robot moves at the speed of its task.
%
% aF, aN   : final and planner (nominal) action (0 = task-layer motion)
% out      : hrcStepHRC output (supervisor.reason, adaptReason, speedReason)
% phase    : task-layer phase;  mode: 0 solo, 1 assistive, 2 collaborative
% apScale  : approach speed factor of liveHRC (< 1: slowed by fatigue)
% supCause : cause of the active protective stop (mem.sup.cause), '' if none
% conf     : confidence of the human-state estimate (O(7)); a slow planner choice while it is below
%            prm.minHumanConfidence is attributed to the low confidence (the unobserved fatigue belief drifts up)
%
% code / label:
%   0 none                 normal speed, or the planned motion of the task
%   1 fatigue level        fatigue adaptation, approach at slow speed, rest warning hold
%   2 fatigue + distance   solo rule: medium / high fatigue and hand closer than 0.30 m
%   3 distance             hand / arm too close, path conflict, separation not verified (solo)
%   4 attention            looking away while near the robot (>= 2 s)
%   5 low confidence       human-state estimate not reliable enough (< 0.5)
%   6 sensing              safety geometry not verified (camera / robot pose missing)
%   7 planner              the POMDP planner itself chose a slower action (belief)
%   8 task / worker        hand outside the work area, worker resting, handover hold
if nargin < 7, supCause = ''; end
if nargin < 8, conf = NaN; end
L = speedCauseLabels();
sr = out.supervisor.reason; ar = out.adaptReason;
spr = ''; if isfield(out,'speedReason'), spr = out.speedReason; end
slowNominal = any(aN == [2 4 6 7 10]);            % planner chose slow / hold / stop
reduced = any(aF == [2 4 6 7 8 10]);              % final action slower than the normal one of its mode
code = 0;
r = sr;
if any(strcmp(sr, {'stop_held_until_verified_clearance','handover_hold_until_hand_clear'})) && ~isempty(supCause)
    r = supCause;                                % a held stop keeps the cause that started it
end
if startsWith(r, 'safety_unverified') || strcmp(r, 'emergency_stop')
    code = 6;
elseif any(strcmp(r, {'active_interaction_risk','hand_prohibited_zone','path_conflict','solo_separation_not_verified_slow'}))
    code = 3;
elseif strcmp(r, 'risky_distraction')
    code = 4;
elseif strcmp(r, 'low_human_state_confidence_slow')
    code = 5;
elseif any(strcmp(r, {'hand_outside_envelope_wait','handover_contact','handover_contact_hold_stationary','handover_hold_until_hand_clear'}))
    code = 8;
elseif contains(ar, 'fatigue')
    code = 1;
elseif strcmp(ar, 'rest_robot_to_safe_position')
    code = 8;
elseif startsWith(spr, 'solo_fatigue')
    code = 2;
elseif aF == 0 && apScale < 1
    code = 1;
elseif reduced && slowNominal && isfinite(conf) && conf < 0.5
    code = 5;                                    % planner slow because the fatigue estimate is missing / unreliable
elseif reduced && slowNominal
    code = 7;
elseif reduced && any(strcmp(phase, {'HOLD','IDLE','GRIP'}))
    code = 8;
end
if ~reduced && code ~= 1 && aF ~= 10, code = 0; end   % moving at its normal speed: nothing reduced it
label = L{code + 1};
end

function L = speedCauseLabels()
L = {'none','fatigue level','fatigue + distance','distance','attention','low confidence', ...
     'sensing','planner (belief)','task / worker'};
end
