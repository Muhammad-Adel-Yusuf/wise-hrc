function [O, ev, geo, ad] = realCameraObservationAdapter(pkt, rxTime, tNow, env, prm, ad)
% Observation of one live decision epoch: REAL worker (camera message) + robot.
% ROBOT: the REAL robot when the camera message carries it (pkt.robot: base,
% shoulder, elbow, tool in W, read from the robot by the camera program; W then
% sits on the real robot base, workspace_calibration.py with the robot-camera
% calibration). Otherwise the VIRTUAL robot (env). The real robot has no planned
% path here: its planned points are its current points (the path check then
% uses the current pose). geo.realRobot tells which robot was used.
% HEAD: pkt.head (W) -> geo.head and geo.headDistance = distance from the head
% point to the robot link centre lines [m] (log / display; not used by the
% planner or the safety supervisor).
% Same outputs as syntheticObservationHRC, so belief, planner and safety
% supervisor are exactly the ones of the simulation.
%
% pkt    : newest camera message (cameraReaderHRC), [] if none yet
% rxTime : reception time of pkt, tNow : current time (same clock) [s]
% env    : virtual robot (syntheticEnvironmentHRC): measured and planned robot
%          points, commanded motion and measured speed come from here
% ad     : adapter memory ([] at the start): previous task counters
% O      : 1x18 observation (index map in stateDefinitionHRC); O(5), O(10),
%          O(16..18) are set by the live loop / task layer
% ev     : behavioural events [backtrack prolongedView hesitation correction]
%          (increments of the camera program's task counters)
% geo    : safety geometry (geometryObservationHRC) of the arm that is CLOSEST to
%          the robot (worst case of both arms), plus geo.arm = 'left' | 'right' | ''
%          and geo.inAssemblyArea (that hand inside the fixed assembly area)
%
% Camera message (mental_fatigue_algorithm/fatigue_monitor/pomdp_bridge.py):
%   Fp, Fm (0..2 or null), attention (0 focused, 1 uncertain, 2 distracted, null),
%   face, person, conf, calibrated, counts.backtracking / hesitation / error,
%   arms.left / arms.right . shoulder / elbow / wrist / palm ([x y z] in frame W or null)
% Safety geometry is VERIFIED only when the workstation frame is calibrated,
% the message is fresh and one arm is complete (shoulder, elbow, wrist, palm with
% depth). An incomplete arm (e.g. a hand without its arm, or an arm without its
% hand), or no message, is NOT verified: the safety supervisor stops the
% virtual robot. When the camera sees nobody, the work cell counts as empty.
% HAND-ONLY SAFETY (prm.handOnlySafety = true, set in liveHRC): when the camera
% cannot see the shoulder / elbow / wrist (e.g. elbows hidden below the table
% edge), an arm with a palm counts as tracked: the missing points are ESTIMATED
% from the palm, the forearm and upper arm pointing from the hand towards the
% worker (+x) - wrist 8 cm, elbow 33 cm behind the palm, shoulder 5 cm further
% and 25 cm higher. Measured points are always used when they arrive. The real
% arm can lie elsewhere (e.g. reaching sideways): this is a simplification for
% the virtual-robot tests, NOT a validated safety function.
% PALM-ONLY SAFETY (prm.palmOnlyClearance = true, set in liveHRC): only the
% measured palm points are used for safety. NO arm points are estimated, a
% tracked palm is enough for a verified geometry, and the clearance is the
% palm sphere (radius prm.geo.rHand) to the robot capsules (geometryHRC).
% Shoulder / elbow / wrist, when the camera sends them, are kept for display
% only. Contact of the upper arm with the robot is NOT checked (stated
% assumption of the virtual-robot tests). palmOnlyClearance overrides
% handOnlySafety.
if isempty(ad)
    ad = struct('counts',[]);
end
O = NaN(1,prm.nObs);
ev = zeros(1,4);
raw = env.raw;                                   % virtual robot: measured + planned points
raw.timestamp = rxTime;
raw.humanInCell = NaN;
raw.handTracked = 0;
raw.limbTracked = 0;
raw.safetyConfidence = 0;
nanArm = struct('shoulder',NaN(1,3),'elbow',NaN(1,3),'wrist',NaN(1,3),'palm',NaN(1,3));
raw.human = nanArm;
arms = {};
names = {};
track = 'no camera message';                     % diagnostics: which arm points arrived
calibrated = false;
palmOnly = isfield(prm,'palmOnlyClearance') && prm.palmOnlyClearance;
handOnly = ~palmOnly && isfield(prm,'handOnlySafety') && prm.handOnlySafety;
estimated = {};
% continuous sensor signals for the plots (NaN = not available)
sig = struct('mentalScore',NaN,'physicalF',NaN,'awayS',NaN,'fatigueReset',NaN);
jointSpeed = NaN;
if ~isempty(pkt)
    % ---------------- worker state
    O(1) = num(pkt,'Fp');
    O(2) = num(pkt,'Fm');
    O(3) = num(pkt,'attention');
    O(4) = double(flag(pkt,'person'));
    O(7) = num(pkt,'conf');
    O(8) = double(flag(pkt,'face'));
    if O(8)==0, O(3) = NaN; end                  % attention is not observed without the face
    sig.mentalScore = num(pkt,'mental_score');   % mental fatigue score 0..1 (levels at 0.4 / 0.7)
    sig.physicalF = num(pkt,'physical_f');       % physical fatigue index 0..1 (levels at 0.3 / 0.6)
    sig.awayS = num(pkt,'away_s');               % time looking away from the work now [s]
    sig.fatigueReset = num(pkt,'fatigue_reset'); % manual fatigue resets of the camera program (keys F, Z Z)
    % mental level briefly invalid (e.g. a few seconds after looking away): the last valid level and its
    % confidence are kept for prm.mentalHoldS seconds (otherwise confidence 0.3 forces slow speed for ~15 s)
    if ~isfield(ad,'Fm'), ad.Fm = NaN; ad.conf = NaN; ad.tFm = -Inf; end
    if ~isnan(O(2))
        ad.Fm = O(2); ad.conf = O(7); ad.tFm = tNow;
    elseif ~isnan(ad.Fm) && tNow - ad.tFm <= prm.mentalHoldS
        O(2) = ad.Fm; O(7) = ad.conf;
    end
    if isfield(pkt,'counts')
        c = [num(pkt.counts,'backtracking') num(pkt.counts,'hesitation') num(pkt.counts,'error')];
        c(isnan(c)) = 0;
        if ~isempty(ad.counts)                   % first message: counters only stored
            inc = max(c - ad.counts, 0) > 0;
            ev = [inc(1) 0 inc(2) inc(3)];
        end
        ad.counts = c;
    end
    % ---------------- upper limbs
    calibrated = flag(pkt,'calibrated');
    track = 'camera program: workstation NOT calibrated';
    if calibrated && isfield(pkt,'arms')
        track = '';
        for side = {'left','right'}
            if isfield(pkt.arms, side{1})
                a = pkt.arms.(side{1});
                h = nanArm;
                seen = '';
                for f = {'shoulder','elbow','wrist','palm'}
                    if isfield(a, f{1}) && numel(a.(f{1}))==3
                        h.(f{1}) = a.(f{1})(:)';
                        seen = [seen ' ' f{1}]; %#ok<AGROW>
                    end
                end
                if isempty(seen), seen = ' -'; end
                if handOnly && all(isfinite(h.palm))
                    [h, est] = armFromPalm(h);
                    if est, estimated{end+1} = side{1}; seen = [seen ' (arm estimated)']; end %#ok<AGROW>
                end
                track = sprintf('%s%s:%s   ', track, side{1}, seen);
                if palmOnly || handOnly
                    keep = all(isfinite(h.palm));        % hand-only / palm-only: an arm counts by its hand
                else
                    keep = any(isfinite([h.shoulder h.elbow h.wrist h.palm]));   % any tracked point counts
                end
                if keep
                    arms{end+1} = h; %#ok<AGROW>
                    names{end+1} = side{1}; %#ok<AGROW>
                end
            end
        end
    end
    if O(4)==0 && isempty(arms)
        raw.humanInCell = 0;                     % camera sees nobody: empty work cell
        if flag(pkt,'calibrated'), raw.safetyConfidence = 0.95; end
    else
        raw.humanInCell = 1;
    end
end
% ---------------- real robot from the camera program (replaces the virtual robot)
realRobot = false;
robotStatus = 'no robot block in the camera message';
if ~isempty(pkt) && isfield(pkt,'robot') && isstruct(pkt.robot) && isfield(pkt.robot,'status')
    robotStatus = char(pkt.robot.status);        % camera program: why there is no fresh robot pose
end
if ~isempty(pkt) && isfield(pkt,'robot') && isstruct(pkt.robot)
    rb = pkt.robot;
    ok = true;
    for f = {'base','shoulder','elbow','tool'}
        ok = ok && isfield(rb, f{1}) && numel(rb.(f{1}))==3 && all(isfinite(rb.(f{1})));
    end
    if ok
        R = struct('base',rb.base(:)','shoulder',rb.shoulder(:)','elbow',rb.elbow(:)','tool',rb.tool(:)');
        raw.robot = R;
        raw.robotPlanned = R;
        sp = num(rb,'speed');
        if isfinite(sp), raw.measuredRobotSpeed = sp; end
        jointSpeed = num(rb,'jspeed');           % measured joint speed [deg/s] (largest joint rate)
        realRobot = true;
    end
end
% ---------------- geometry: worst case of the tracked arms
geo = [];
best = Inf;
for i = 1:numel(arms)
    r = raw;
    r.human = arms{i};
    if palmOnly
        full = all(isfinite(arms{i}.palm));      % palm-only: the palm alone is trusted
    else
        full = all(isfinite([arms{i}.shoulder arms{i}.elbow arms{i}.wrist arms{i}.palm]));
    end
    r.handTracked = double(all(isfinite(arms{i}.palm)));
    r.limbTracked = double(full);
    r.safetyConfidence = 0.95*full;              % only complete arms (or palms, palm-only) are trusted
    gi = geometryObservationHRC(r, tNow, prm);
    if full, key = min(gi.minUpperLimbClearance, gi.handInteractionDistance); else, key = -Inf; end
    if isempty(geo) || key < best                % a hand without its arm wins: not verified
        geo = gi; best = key; geo.arm = names{i};
    end
end
if isempty(geo)
    geo = geometryObservationHRC(raw, tNow, prm);
    geo.arm = '';
end
geo.realRobot = realRobot;
geo.robotStatus = robotStatus;
geo.sig = sig;
geo.measuredJointSpeed = jointSpeed;
% ---------------- head: distance to the robot links (centre lines)
geo.head = NaN(1,3);
geo.headDistance = NaN;
if ~isempty(pkt) && isfield(pkt,'head') && numel(pkt.head)==3 && all(isfinite(pkt.head))
    geo.head = pkt.head(:)';
    r = raw.robot;
    seg = {r.base, r.shoulder; r.shoulder, r.elbow; r.elbow, r.tool};
    d = Inf;
    for j = 1:3
        d = min(d, segmentDistanceHRC(geo.head, geo.head, seg{j,1}, seg{j,2}));
    end
    geo.headDistance = d;
end
% ---------------- clearer reason when the geometry is not verified
geo.track = strtrim(track);
geo.armEstimated = ~isempty(geo.arm) && any(strcmp(estimated, geo.arm));
if ~geo.verified && geo.fresh
    if ~calibrated
        geo.unverifiedReason = 'camera_program_not_calibrated';
    elseif isempty(arms) && (handOnly || palmOnly)
        geo.unverifiedReason = 'no_hand_from_camera';
    elseif isempty(arms)
        geo.unverifiedReason = 'no_arm_points_from_camera';
    elseif ~isempty(geo.arm) && ~palmOnly       % palm-only: missing arm points do not matter
        h = geo.human; f = {'shoulder','elbow','wrist','palm'}; miss = '';
        for j = 1:4
            if ~all(isfinite(h.(f{j}))), miss = [miss '_' f{j}]; end %#ok<AGROW>
        end
        if ~isempty(miss)
            geo.unverifiedReason = ['arm_incomplete_' geo.arm '_missing' miss];
        end
    end
end
% ---------------- hand inside the fixed assembly area (for display and log)
geo.inAssemblyArea = false;
pa = geo.human.palm;
if all(isfinite(pa))
    A = prm.assemblyArea;
    geo.inAssemblyArea = pa(1) >= A.x(1) && pa(1) <= A.x(2) && pa(2) >= A.y(1) && pa(2) <= A.y(2) ...
        && pa(3) <= A.height + 0.10;
end
% ---------------- safety part of O (copies of geo)
O(6) = geo.risk;
O(9) = geo.zone;
O(10) = 0;
O(11) = geo.pathConflict;
O(12) = geo.safetyConfidence;
O(13) = geo.age;
O(14) = geo.handInteractionDistance;
O(15) = geo.minUpperLimbClearance;
O(16) = 0;
O(17) = NaN;
O(18) = 0;
end

function [h, est] = armFromPalm(h)
% hand-only safety: missing wrist / elbow / shoulder estimated from the palm
% (arm pointing from the hand towards the worker, +x in frame W)
est = false;
P = h.palm(:)';
guess = struct('wrist', P + [0.08 0 0], 'elbow', P + [0.33 0 0], 'shoulder', P + [0.38 0 0.25]);
for f = {'wrist','elbow','shoulder'}
    if ~all(isfinite(h.(f{1})))
        h.(f{1}) = guess.(f{1});
        est = true;
    end
end
end

function v = num(s, f)
% numeric field, NaN when missing or null
v = NaN;
if isfield(s, f) && ~isempty(s.(f)) && (isnumeric(s.(f)) || islogical(s.(f)))
    v = double(s.(f)(1));
end
end

function tf = flag(s, f)
tf = isfield(s, f) && ~isempty(s.(f)) && logical(s.(f)(1));
end
