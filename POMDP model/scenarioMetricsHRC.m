function [m, ev] = scenarioMetricsHRC(rec, prm)
% Numbers and event list of ONE recorded scenario (G ... X) for the paper.
% [m, ev] = scenarioMetricsHRC(rec, prm)
%   rec : recording of liveHRC (rec.data, rec.cols, rec.reason, rec.adapt, rec.phase)
%   prm : stateDefinitionHRC of the run (distances, decision period)
%   m   : struct of scalar metrics (field names = CSV names, units in the name)
%   ev  : table of events in time order: time_s, event, robot action, reason
% Recordings made before a column existed give NaN for the metrics that need it.
%
% Metrics
%   duration, share of time at normal / slow / stopped / protective stop
%   protective stops: number, by cause (proximity = hand or arm too close /
%     path conflict, sensing = geometry not verified, distraction, emergency
%     stop), total time
%   distances: minimum hand-tool distance and arm clearance (all time and
%     while the robot moves), time in each hand zone
%   safety checks (should be 0): robot moving with the hand inside the
%     prohibited distance; robot at normal speed with the hand in the
%     prohibited or collaborative zone
%   reaction: hand crosses the prohibited distance -> protective stop command;
%     latency = decision epochs between them + age of the camera message
%     (the camera's own delay before sending is NOT included)
%   sensing: geometry verified %, arm estimated from the hand %, camera
%     message age and rate
%   worker state: % of time each camera reading was available, mean levels,
%     highest monitor level, rest warnings, rests, human-state confidence
%   assistive: tool requests, deliveries, request -> tool taken time
D = rec.data; c = rec.cols;
n = size(D,1);
g = @(name) col(D, c, name, n);
t = g('t'); dt = prm.dt;
ev = table(zeros(0,1), cell(0,1), cell(0,1), cell(0,1), ...
    'VariableNames', {'time_s','event','robot_action','reason'});
m = struct();
m.name = rec.name;
modes = {'solo','assistive','collaborative'};
mo = unique(g('mode')); mo = mo(~isnan(mo));
if isscalar(mo), m.mode = modes{mo+1}; else, m.mode = 'mixed'; end
if n < 2, return; end
reason = textCol(rec, 'reason', n);
aF = g('aFraw'); if all(isnan(aF)), aF = g('aF'); end
aPlot = g('aF');
mot = g('motion');
% ---------------- time and motion
m.duration_s = t(end) - t(1) + dt;
m.share_normal_pct = 100*mean(mot==2);
m.share_slow_pct = 100*mean(mot==1);
m.share_stopped_pct = 100*mean(mot==0);
m.share_protective_stop_pct = 100*mean(mot==-1);
% ---------------- protective stops
isPS = aF==10;
on = find(isPS & [true; ~isPS(1:end-1)]);
cause = cell(numel(on),1);
for i = 1:numel(on), cause{i} = stopCause(reason{on(i)}); end
m.stops_total = numel(on);
m.stops_proximity = sum(strcmp(cause,'proximity'));
m.stops_sensing = sum(strcmp(cause,'sensing'));
m.stops_distraction = sum(strcmp(cause,'distraction'));
m.stops_emergency = sum(strcmp(cause,'emergency'));
m.stops_other = sum(strcmp(cause,'other'));
m.stop_time_s = sum(isPS)*dt;
% ---------------- distances and zones
dH = g('dHand'); cl = g('clearance'); sp = g('speed'); zn = g('zone');
moving = sp > prm.stationarySpeed;
m.min_hand_tool_dist_m = minNaN(dH);
m.min_hand_tool_dist_moving_m = minNaN(dH(moving));
m.min_arm_clearance_m = minNaN(cl);
m.min_arm_clearance_moving_m = minNaN(cl(moving));
m.time_zone_prohibited_s = sum(zn==0)*dt;
m.time_zone_collaborative_s = sum(zn==1)*dt;
m.time_zone_assistive_s = sum(zn==2)*dt;
m.time_zone_separated_s = sum(zn==3)*dt;
m.time_zone_unknown_s = sum(isnan(zn))*dt;
m.check_moving_in_prohibited_epochs = sum(moving & dH < prm.dProhibit);
m.check_normal_speed_hand_close_epochs = sum(mot==2 & zn<=1);
% ---------------- reaction to the hand crossing the prohibited distance
age = g('age');
inP = dH < prm.dProhibit;
cross = find(inP & [true; ~inP(1:end-1)]);
lat = NaN(numel(cross),1);
for i = 1:numel(cross)
    j = find(isPS(cross(i):end), 1);
    if ~isempty(j)
        k = cross(i) + j - 1;
        lat(i) = t(k) - t(cross(i)) + max(age(cross(i)), 0);
    end
end
m.prohibited_crossings = numel(cross);
m.reaction_mean_s = meanNaN(lat);
m.reaction_max_s = maxNaN(lat);
m.decision_period_s = dt;
% ---------------- sensing
m.geometry_verified_pct = 100*mean(g('verified')==1);
m.arm_estimated_pct = 100*mean(g('armEst')==1);
m.camera_age_mean_s = meanNaN(age);
m.camera_age_max_s = maxNaN(age);
rc = g('rxCount');
m.camera_msg_rate_hz = (rc(end) - rc(1))/max(t(end) - t(1), eps);
% ---------------- worker state
Fp = g('Fp'); Fm = g('Fm'); A = g('A');
m.Fp_available_pct = 100*mean(~isnan(Fp));
m.Fm_available_pct = 100*mean(~isnan(Fm));
m.attention_available_pct = 100*mean(~isnan(A));
m.Fp_mean = meanNaN(Fp);
m.Fm_mean = meanNaN(Fm);
m.attention_mean = meanNaN(A);
m.distracted_time_s = sum(A==2)*dt;
m.fatigue_level_max = maxNaN(g('level'));
dg = g('dialog');
m.rest_warnings = sum(dg==1 & [true; dg(1:end-1)~=1]);
m.rests = sum(dg==3 & [true; dg(1:end-1)~=3]);
m.human_confidence_mean = meanNaN(g('conf'));
% ---------------- assistive: tool requests
req = g('requests'); del = g('deliveries');
m.tool_requests = req(end) - req(1);
m.tool_deliveries = del(end) - del(1);
hand = NaN(0,1);
kd = find(diff(del) > 0) + 1;
for i = 1:numel(kd)
    kr = find(diff(req(1:kd(i))) > 0, 1, 'last') + 1;
    if ~isempty(kr), hand(end+1,1) = t(kd(i)) - t(kr); end %#ok<AGROW>
end
m.request_to_taken_mean_s = meanNaN(hand);
% ---------------- events (time order)
E = {};
add = @(k, txt) {t(k), txt, actionName(aPlot(k)), reason{k}};
btn = g('button');
mk = g('mode');
E(end+1,:) = add(1, 'START (G)');
for k = 2:n
    if btn(k)==2, E(end+1,:) = add(k, 'PAUSE (P)'); end %#ok<AGROW>
    if btn(k)==1 || btn(k)==3, E(end+1,:) = add(k, 'RESUME (R)'); end %#ok<AGROW>
    if req(k) > req(k-1), E(end+1,:) = add(k, 'tool request (T)'); end %#ok<AGROW>
    if del(k) > del(k-1), E(end+1,:) = add(k, 'tool taken'); end %#ok<AGROW>
    if g1(dg,k)==1 && g1(dg,k-1)~=1, E(end+1,:) = add(k, 'rest warning'); end %#ok<AGROW>
    if g1(dg,k)==3 && g1(dg,k-1)~=3, E(end+1,:) = add(k, 'rest started'); end %#ok<AGROW>
    if g1(dg,k-1)==3 && g1(dg,k)~=3, E(end+1,:) = add(k, 'rest ended'); end %#ok<AGROW>
    if mk(k)~=mk(k-1), E(end+1,:) = add(k, ['mode -> ' modes{mk(k)+1}]); end %#ok<AGROW>
    if isPS(k) && ~isPS(k-1), E(end+1,:) = add(k, ['protective stop (' stopCause(reason{k}) ')']); end %#ok<AGROW>
    if ~isPS(k) && isPS(k-1), E(end+1,:) = add(k, 'protective stop released'); end %#ok<AGROW>
    if inP(k) && ~inP(k-1), E(end+1,:) = add(k, sprintf('hand inside prohibited distance (%.2f m)', dH(k))); end %#ok<AGROW>
    if aPlot(k)~=aPlot(k-1), E(end+1,:) = add(k, 'action change'); end %#ok<AGROW>
end
if btn(n)==5, E(end+1,:) = add(n, 'STOP (X)'); end
if ~isempty(E)
    ev = cell2table(E, 'VariableNames', {'time_s','event','robot_action','reason'});
end
end

% =========================================================================
function v = col(D, c, name, n)
if isfield(c, name), v = D(:, c.(name)); else, v = NaN(n,1); end
end

function v = g1(x, k)
v = x(k);
end

function r = textCol(rec, f, n)
if isfield(rec, f) && numel(rec.(f))==n
    r = rec.(f)(:);
else
    r = repmat({''}, n, 1);
end
end

function s = stopCause(reason)
if startsWith(reason, 'safety_unverified')
    s = 'sensing';
elseif any(strcmp(reason, {'active_interaction_risk','hand_prohibited_zone','path_conflict'}))
    s = 'proximity';
elseif strcmp(reason, 'risky_distraction')
    s = 'distraction';
elseif strcmp(reason, 'emergency_stop')
    s = 'emergency';
else
    s = 'other';
end
end

function s = actionName(a)
names = {'task-layer move','solo normal','solo slow','approach','assistive hold','collab. normal', ...
    'collab. slow','collab. hold','assist / realloc.','retreat','protective stop','wait at safe position'};
if isnan(a), s = ''; else, s = names{a+1}; end
end

function v = minNaN(x)
x = x(~isnan(x)); if isempty(x), v = NaN; else, v = min(x); end
end

function v = maxNaN(x)
x = x(~isnan(x)); if isempty(x), v = NaN; else, v = max(x); end
end

function v = meanNaN(x)
x = x(~isnan(x)); if isempty(x), v = NaN; else, v = mean(x); end
end
