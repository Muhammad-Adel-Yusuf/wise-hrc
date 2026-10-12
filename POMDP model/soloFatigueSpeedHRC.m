function [aF, scale, reason, st] = soloFatigueSpeedHRC(aF, M, geo, level, st, prm, attention)
% SOLO SPEED ADAPTATION TO FATIGUE AND DISTANCE (after the safety supervisor).
% It never releases a stop of the supervisor. Apart from the low-fatigue rule
% below it only makes the robot slower or stops it.
%
% aF    : action after the safety supervisor
% M     : task mode (only solo, M = 0, is adapted)
% geo   : observed safety geometry (geo.minUpperLimbClearance = distance of the
%         tracked hand / limb to the nearest robot part [m])
% level : sustained fatigue level of the monitor (0 low, 1 medium, 2 high, NaN unknown)
% st    : memory ([] at the start), st.stopped = 1 while this rule holds a stop
% attention : observed attention O(3) (0 focused, 1 uncertain, 2 distracted, NaN unknown)
% scale : speed factor for the robot (1 = the speed of the action file)
%
% Rule (prm.soloSpeed):
%   low fatigue, separation verified (clearance >= prm.cSeparated = 0.30 m) and
%     attention focused: NORMAL speed (1), also when the planner chose slow
%     because the palm is inside its 0.35 m collaborative zone (user choice
%     2026-10-09: same speed at 0.40 and 0.30 m for a rested worker)
%   low fatigue otherwise, or distance >= dSlow: unchanged
%   medium / high fatigue and distance < dSlow: solo SLOW (2) with its speed
%     scaled by  r^p,  r = (d - dStop) / (dSlow - dStop)  clipped to 0..1,
%     dStop = prm.cActive (0.05 m, active-risk clearance next to the
%     prohibited zone), p = exponent(level): medium 1 (linear), high 2 (the
%     speed falls faster). At dSlow the speed is the normal slow speed.
%   scale below minScale: PROTECTIVE STOP (10), held until the scale is
%     above resumeScale again (hysteresis).
if isempty(st), st = struct('stopped', false); end
if nargin < 7, attention = NaN; end
scale = 1; reason = '';
c = prm.soloSpeed;
if M == 0 && level == 0 && aF == 2 && geo.separationVerified && attention == 0
    aF = 1; st.stopped = false;
    reason = sprintf('solo_low_fatigue_separated_normal (%.2f m)', geo.minUpperLimbClearance);
    return
end
if M ~= 0 || isnan(level) || level < 1 || ~any(aF == [1 2])
    st.stopped = false;
    return
end
d = geo.minUpperLimbClearance;
if ~isfinite(d) || d >= c.dSlow                 % Inf: verified empty cell; NaN: the supervisor already stops
    st.stopped = false;
    return
end
r = min(max((d - c.dStop) / (c.dSlow - c.dStop), 0), 1);
scale = r ^ c.exponent(level + 1);
thr = c.minScale;
if st.stopped, thr = c.resumeScale; end
if scale < thr
    aF = 10; scale = 0; st.stopped = true;
    reason = sprintf('solo_fatigue_proximity_stop (fatigue %d, %.2f m)', level, d);
else
    aF = 2; st.stopped = false;
    reason = sprintf('solo_fatigue_slow %.0f%% (fatigue %d, %.2f m)', 100 * scale, level, d);
end
end
