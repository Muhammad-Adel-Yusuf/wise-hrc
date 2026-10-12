function prm = stateDefinitionHRC()
% State definition and parameters of the hierarchical POMDP HRC model
% (planner = QMDP approximation of the POMDP, see pomdpActionHRC)
% The parameters are built once and kept (persistent); "clear stateDefinitionHRC"
% rebuilds them (liveHRC does this at its start, e.g. after a new calibration).
% LIVE WORKSTATION CALIBRATION: if workspace_calibration.json (written by
% mental_fatigue_algorithm/workspace_calibration.py) is in this folder, the
% places clicked on the camera image are used: robot base, assembly area,
% handover point, tool storage, safe position, and the collaborative / assistive
% distances sized from them (see applyWorkspaceCalibration at the end of this file).
%
% S = [M I H Fp Fm A Z V R]
%   M  task mode          0 solo-independent, 1 assistive, 2 collaborative
%   I  instruction        1..6
%   H  human presence     0 absent, 1 present
%   Fp physical fatigue   0 low, 1 medium, 2 high      (hidden)
%   Fm mental fatigue     0 low, 1 medium, 2 high      (hidden)
%   A  attention          0 focused, 1 uncertain, 2 distracted (hidden)
%   Z  HAND interaction zone, classified from handInteractionDistance only
%      (zoneClassifyHRC):
%        0 hand violates the prohibited interaction boundary
%        1 hand inside the collaborative envelope
%        2 hand inside the assistive region
%        3 hand in the separated region
%      Z says nothing about the elbow, forearm or upper arm: Z == 3 does NOT
%      prove whole-arm separation.
%   V  commanded robot motion   0 stopped, 1 slow, 2 normal
%      (the measured robot speed is a separate observation, geo.measuredRobotSpeed)
%   R  interaction risk (riskEstimateHRC), estimated from the whole upper-limb
%      clearance, robot speed, path conflict, hazard signals and the hand zone:
%        0 no detected risk, 1 potential risk, 2 active safety risk
% tau (risky-inattention time) is NOT in S, it is kept by safetySupervisorHRC.
%
% Two kinds of geometry are kept apart:
%   - PREDICTED geometry: Z and R inside the planner model (NextStateHRC /
%     TransProbHRC), probabilistic, used only to rank actions
%   - OBSERVED geometry: the struct geo (geometryObservationHRC), measured
%     each cycle, used by the safety supervisor and as the planner context
%
% Observation vector O (one row per decision epoch), index map in prm.obs:
%   O(1)  physicalFatigue        0..2 or NaN
%   O(2)  mentalFatigue          0..2 or NaN
%   O(3)  attentionState         0..2 or NaN (NaN when the face is not detected)
%   O(4)  humanPresence          0/1
%   O(5)  currentInstruction     1..6
%   O(6)  interactionRisk R      0..2, NaN = unknown   (copy of geo.risk)
%   O(7)  humanStateConfidence   0..1  (fatigue/attention estimation only)
%   O(8)  faceDetected           0/1
%   O(9)  handZone Z             0..3, NaN = unknown   (copy of geo.zone)
%   O(10) emergencyStop          0/1
%   O(11) pathConflict           0/1, NaN = unknown    (copy of geo.pathConflict)
%   O(12) safetyConfidence       0..1  (geometry / safety sensing only)
%   O(13) geometryAge [s]        time since the geometry was measured
%   O(14) handInteractionDistance [m]
%   O(15) minUpperLimbClearance [m]
%   O(16) handoverActive         0/1  task layer: robot holds a tool at the handover point
%   O(17) toolTaken              0/1, NaN = no signal  (camera detection or keyboard confirmation)
%   O(18) humanResponse          worker button (keyboard now; camera / voice later)
%                                0 or NaN none, 1 RESUME (continue / back to work),
%                                2 PAUSE (rest now), 3 same as RESUME,
%                                4 START session, 5 STOP session (not an emergency stop)
% The named struct geo (geometryObservationHRC) carries the same safety data
% with full detail and is what the safety supervisor reads.
% Behavioural events ev = [backtrack prolongedView hesitation correction] (0/1)
%
% Geometry convention (synthetic workstation, see syntheticEnvironmentHRC)
%   frame W : origin at the robot base on the table top, x from the robot
%             towards the seated human, y to the human's right, z up
%   units   : meters, seconds
%   human   : right upper limb as 3 capsules (shoulder-elbow, elbow-wrist,
%             wrist-palm); palm = hand reference point
%   robot   : 3 capsules (base column, upper link, forearm link + tool);
%             tool tip = robot interaction reference (tool / handover point)
% ALL DISTANCES AND THRESHOLDS BELOW ARE ILLUSTRATIVE. THEY ARE NOT VALIDATED
% SAFETY DISTANCES AND MUST NOT BE USED ON A REAL ROBOT.

persistent cache
if ~isempty(cache)
    prm = cache;
    return
end
% ------------------------------------------------------------ sizes
prm.nM = 3; prm.nI = 6; prm.nH = 2; prm.nF = 3; prm.nA = 3;
prm.nZ = 4; prm.nV = 3; prm.nR = 3;
prm.nS = 3*6*2*3*3*3*4*3*3;            % 34992 states
prm.nAct = 10;                          % actions
prm.nOut = 13;                          % successor outcomes per action (see NextStateHRC)
prm.dt = 0.5;                           % decision epoch [s]
prm.gamma = 0.95;

% ------------------------------------------------------------ observation index map
prm.obs.physicalFatigue = 1;  prm.obs.mentalFatigue = 2;   prm.obs.attention = 3;
prm.obs.presence = 4;         prm.obs.instruction = 5;     prm.obs.risk = 6;
prm.obs.humanConfidence = 7;  prm.obs.faceDetected = 8;    prm.obs.zone = 9;
prm.obs.emergencyStop = 10;   prm.obs.pathConflict = 11;   prm.obs.safetyConfidence = 12;
prm.obs.geometryAge = 13;     prm.obs.handDistance = 14;   prm.obs.clearance = 15;
prm.obs.handoverActive = 16;  prm.obs.toolTaken = 17;   prm.obs.humanResponse = 18;
prm.nObs = 18;

% ------------------------------------------------------------ task
prm.difficulty   = [0.2 0.4 0.7 0.5 0.8 0.3];  % instruction difficulty (0..1)
prm.assistNeeded = [0 1 0 1 0 0];               % instructions that need robot help (assistive mode)
prm.homeZone  = [3 2 1];   % preferred HAND zone of solo / assistive / collaborative mode
prm.waitAction = [2 4 7];  % stopped/slow waiting action of each mode (supervisor fallback)

% ------------------------------------------------------------ human transitions (unchanged)
prm.physUp = 0.03;  prm.physDown = 0.02;    % max prob. of one-level change per epoch
prm.mentUp = 0.03;  prm.mentDown = 0.02;
prm.physBase = 0.40; prm.physTime = 0.30;   % physical workload
prm.mentBase = 0.25; prm.mentTime = 0.25;   % mental workload
prm.kDiff = 0.25; prm.kBack = 0.15; prm.kView = 0.10; prm.kHes = 0.10; prm.kCorr = 0.10;
prm.restFactor = 2.0;                        % recovery gain during pauses (actions 4, 7, 10)
prm.absentRecovery = 3.0;                    % recovery gain when the human is absent
%                   1     2     3     4     5     6     7     8     9    10
prm.robotPhys  = [0.00  0.00  0.00 -0.20  0.15 -0.10 -0.25 -0.30 -0.10 -0.25];
prm.robotMent  = [0.00  0.00  0.05 -0.15  0.10 -0.05 -0.15 -0.25 -0.05 -0.15];
prm.soloNear = 0.10;                         % solo robot moving while the hand is not separated
prm.attWorse  = [0.08 0.15 0.00];            % P(attention gets worse) from focused/uncertain/distracted
prm.attBetter = [0.00 0.30 0.25];            % P(attention gets better)
prm.attConfirm = 0.30;                       % hold-and-confirm pulls attention back to focus
prm.evExpected = [0.04 0.08 0.06 0.04];      % expected event rates used for planning
prm.timePlan = 0.5;                          % time on task used for planning
prm.instrAdvance = 0.06; prm.instrBack = 0.01; prm.backFatigue = 0.02;
prm.advFatigueLoss = 0.30; prm.advDistractLoss = 0.50;
prm.assistGain = 0.30; prm.holdFactor = 0.30;

% ------------------------------------------------------------ PREDICTED geometry (planner model)
% per-epoch probabilities used only by NextStateHRC / TransProbHRC
prm.intrusion = [0.002 0.006 0.020];         % hand crosses the prohibited boundary, robot moving, hand Z = 1/2, by attention
prm.intrusionStopped = 0.001;                % same with the robot stopped
prm.sepIntrusion = 0.001;                    % same with the hand separated
prm.limbHazard = [0.002 0.005 0.015];        % elbow/forearm/upper-arm hazard onset, robot moving, hand Z = 1/2, by attention
prm.limbHazardSep = 0.0005;                  % same with the hand separated
prm.pApproach = 0.20;                        % approach (3) brings the hand into the assistive band per epoch
prm.pRetreat  = 0.20;                        % retreat (9) brings the hand into the separated band per epoch
prm.pWithdraw = 0.25;                        % environmental clearance of an unsafe condition per epoch
                                             % (the same for every action: stopping does not cause it)
prm.pRiskDecay = 0.30;                       % residual potential risk disappears (hand separated)
prm.pLeaveEnvelope = 0.01;                   % collaborative hand leaves the envelope
prm.pReturnEnvelope = 0.30;                  % collaborative hand comes back to the envelope

% ------------------------------------------------------------ cost (unchanged)
prm.wPhysical = 0.5; prm.wMental = 0.5;
prm.lambda = [0 0.6 1.0];                    % solo, assistive (lambda_a), collaborative (lambda_c)
prm.timeCost = 0.10; prm.holdCost = 0.10;
prm.stopCost = 3.0; prm.zoneCost = 1.0;
prm.motionIntensity = [0 0.3 1.0];           % stopped, slow, normal
prm.smoothCost = 0.05;
prm.riskMotion = [0 0.05 0.15];
prm.attMotion = [0 0.40 1.20];
prm.safetyCost = 50.0;
% solo mode (2026-10-08): extra cost of robot motion near the human (hand not
% separated) by fatigue level max(Fp, Fm) = low / medium / high. Low: none (the
% solo behaviour of a rested worker is unchanged); medium / high: the planner
% prefers solo slow near the human (normal - slow = 0.7*w > taskLoss 0.30).
prm.soloFatigueMotion = [0 0.6 1.0];
prm.noHumanCost = 1.0;
%               1     2     3     4     5     6     7     8     9    10
prm.taskLoss = [0.00  0.30  1.00  1.00  1.00  1.00  1.00  0.50  0.20  1.00;   % solo
                1.00  1.00  0.10  0.60  1.00  0.30  1.00  0.50  0.20  1.00;   % assistive
                1.00  1.00  1.00  1.00  0.00  0.30  0.80  0.35  1.00  1.00];  % collaborative

% ------------------------------------------------------------ human-state observation (unchanged)
prm.confPhys = [0.80 0.18 0.02; 0.12 0.76 0.12; 0.02 0.18 0.80];   % rows true, cols observed
prm.confMent = [0.80 0.18 0.02; 0.12 0.76 0.12; 0.02 0.18 0.80];
prm.confAtt  = [0.85 0.12 0.03; 0.15 0.70 0.15; 0.05 0.15 0.80];
pF = [0.70 0.25 0.05]; pA = [0.80 0.15 0.05];
prm.prior = zeros(27,1);                     % initial belief, order of hidden index
for fp = 0:2
    for fm = 0:2
        for a = 0:2
            prm.prior(9*fp + 3*fm + a + 1) = pF(fp+1)*pF(fm+1)*pA(a+1);
        end
    end
end
prm.z = 0;                                   % voting bonus of pomdpActionHRC (0 = pure QMDP)

% ------------------------------------------------------------ zone boundaries (ILLUSTRATIVE)
% Z from the palm-to-interaction-reference distance d, first rule that
% matches wins (precedence 0 > 1 > 2 > 3, boundaries belong to the lower zone):
%   d <  dProhibit            -> 0
%   d <= dCollabMax           -> 1
%   d <= dAssistMax           -> 2
%   otherwise                 -> 3
prm.dProhibit  = 0.01;
prm.dCollabMax = 0.15;
prm.dAssistMax = 0.65;
% fixed assistive zone (live tests): circle around the handover point instead of
% the distance dAssistMax from the robot tool (zoneClassifyHRC). Used only when a
% workstation calibration is loaded; the centre is the calibrated handover point.
prm.assistZone = struct('enabled', false, 'useWithCalibration', true, ...
    'radius', 0.065, 'centre', [NaN NaN NaN]);    % radius [m], top view (the workstation file overrides it)

% ------------------------------------------------------------ risk thresholds (ILLUSTRATIVE)
prm.cActive    = 0.05;   % minimum upper-limb clearance for active risk (plus speed term)
prm.tStop      = 0.20;   % [s] stopping-time term: active if clearance < cActive + speed*tStop
prm.cWarn      = 0.25;   % clearance below which risk is potential
prm.cSeparated = 0.30;   % whole-limb clearance that verifies separation (solo continuation; was 0.40, 2026-10-08)
prm.pathHorizon = 1.0;   % [s] look-ahead of the planned robot path for path-conflict checks
prm.stationarySpeed = 0.005;   % [m/s] measured tool speed below which the robot counts as stationary
% solo speed adaptation (soloFatigueSpeedHRC, 2026-10-08): medium / high fatigue and the
% hand closer than dSlow -> slow speed x ((d - dStop)/(dSlow - dStop))^exponent, stop below minScale
% approaches (handover point: action 3; collaborative work point: MOVE in collaborative mode) are commanded at
% NORMAL speed and scaled by the sustained fatigue level low / medium / high (2026-10-09): x1 = normal 23 deg/s,
% x5/23 = slow 5 deg/s. Unknown level (NaN, before the baseline) counts as medium.
prm.approachSpeedScale = [1 5/23 5/23];        % normal 23 deg/s (2026-10-09), slow 5 deg/s
prm.mentalHoldS = 20;                          % [s] last valid mental level kept while the camera reports it invalid
prm.soloSpeed = struct('dSlow', prm.cSeparated, 'dStop', prm.cActive, ...
    'exponent', [NaN 1 2], 'minScale', 0.10, 'resumeScale', 0.20);   % exponent by level low / medium / high
% hysteresis for releasing a protective stop
prm.cRelease    = 0.06;  % clearance needed before release (> cActive 0.05; was 0.10, lowered 2026-10-08 for collaborative work)
prm.handRelease = 0.15;  % hand distance needed before release (> dProhibit; was 0.20, lowered 2026-10-08)

% ------------------------------------------------------------ safety sensing
prm.maxGeometryAge      = 0.25;  % [s] older geometry is stale
prm.minSafetyConfidence = 0.80;  % geometry confidence needed to verify safety
prm.minReleaseConfidence = 0.90; % geometry confidence needed to release a stop
prm.minHumanConfidence  = 0.50;  % human-state confidence below which collaborative speed is reduced

% ------------------------------------------------------------ safety supervisor
prm.distractionLimit = 2.0;                  % [s]
prm.faceLossGrace = 1.0;                     % [s] a single face-detection failure is ignored
prm.releaseSteps = 2;                        % verified all-clear epochs before a stop is released

% ------------------------------------------------------------ fatigue monitor (humanStateMonitorHRC)
% Works on the BELIEF, never on a single reading. Not part of the planner
% model: changing these values does NOT require running valueIterationHRC again.
prm.fatigue.highProb = 0.60;     % belief P(Fp = high) or P(Fm = high) above this -> "high now"
prm.fatigue.medProb  = 0.60;     % belief P(F >= medium) above this -> "medium now"
prm.fatigue.highHold = 30;       % [s] "high now" must last this long before the level becomes high
prm.fatigue.medHold  = 10;       % [s] same for medium
prm.fatigue.downHold = 10;       % [s] a lower value must last this long before the level goes down (was 30)
prm.fatigue.snoozeContinue = 300;% [s] no new rest warning after the worker answered "continue"
prm.fatigue.snoozeRest = 60;     % [s] no new rest warning right after a rest
prm.fatigue.maxRest = Inf;       % [s] rest ends automatically after this time (Inf = keyboard only)
prm.fatigue.struggleWindow = 60; % [s] window for counting struggle events (hesitation, re-reading, corrections)
prm.fatigue.struggleEvents = 3;  % events in the window that mark "struggling"
prm.session.autoStart = true;    % true: the planner runs from the first step (tests);
                                 % false: it waits for the START button (real sessions)

% ------------------------------------------------------------ synthetic workstation geometry (ILLUSTRATIVE)
g.robotBase     = [0 0 0];
g.robotShoulder = [0 0 0.35];
g.robotElbowLift = 0.25;                     % robot elbow = mid(shoulder,tool) + [0 0 lift]
g.rColumn = 0.08; g.rUpperLink = 0.06; g.rForearmTool = 0.05;
g.humanShoulder     = [1.05 0.20 0.45];
g.humanShoulderLean = [0.80 0.12 0.50];      % leaning towards the robot
g.upperArmLength = 0.30; g.forearmLength = 0.27; g.palmToWrist = 0.08;
g.rUpperArm = 0.05; g.rForearm = 0.04; g.rHand = 0.04;
g.elbowPoleDown   = [0 0 -1];                % normal elbow posture
g.elbowPoleRaised = [-0.6 -0.2 0.75];        % raised elbow towards the robot
g.palmWork  = [0.80 0.10 0.10];              % hand working position
g.palmRest  = [0.95 0.32 0.05];              % hand resting beside the workpiece
g.palmReach = [0.12 0.24 0.32];              % reaching for a part next to the robot base
g.intrudeDistance = 0.11;                    % palm-to-tool distance during an intrusion
g.poseSolo   = [0.20 -0.50 0.20];            % robot tool pose of the solo task = SAFE position
g.poseStorage = [0.05 -0.65 0.10];           % tool storage, next to the safe position (separated)
g.graspDistance = 0.10;                      % palm-to-tool distance when the human grasps the tool
g.poseAssist = [0.36 0.10 0.22];             % assistive handover point
g.poseCollab = [0.58 0.10 0.16];             % collaborative work point (fixed, robot never chases the hand)
g.entryPose = {g.poseSolo, g.poseSolo, g.poseCollab};   % pose reached by the transfer phase of each mode
g.robotSpeed = [0 0.04 0.12];                % [m/s] stopped, slow, normal (as the real robot: robot_code.py 5 / 15 deg/s)
g.handSpeed = 0.60;                          % [m/s] max palm speed
g.workAmplitude = 0.02;                      % [m] small working motion around a pose
g.positionNoise = 0.005;                     % [m] measurement noise (std)
prm.geo = g;

% ------------------------------------------------------------ synthetic data
prm.seed = 42;
prm.nSteps = 1200;                           % 1200 x 0.5 s = 10 min
prm.accuracy = 0.80;                         % classifier accuracy
prm.faceDrop = 0.05; prm.faceAway = 0.40;
prm.dipProb = 0.01; prm.dipLen = 6;
prm.presenceNoise = 0.01;
prm.modeSchedule = [1 400 0; 401 800 1; 801 1200 2];      % [start end mode]
% scenario events [start end] in epochs (explicit environmental events)
sc.forcedDistraction = [150 190; 1000 1010];  % attention
sc.lowHumanConfidence = [220 240];            % poor face/fatigue estimation only
sc.handReach    = [280 300];                  % solo: palm separated, forearm near the robot base
sc.staleGeometry = [340 344];                 % geometry not refreshed
sc.trackingLoss = [600 604];                  % hand / limb tracking lost
sc.handRest     = [880 905];                  % collaborative: hand leaves the envelope
sc.handIntrude  = [950 965];                  % hand enters the prohibited region and STAYS there
                                              % until the window ends (withdrawal = end of window)
sc.elbowRaise   = [1050 1060];                % palm in the envelope, elbow/upper arm hazard
sc.emergencyStop = [1100 1102];
sc.lowSafetyConfidence = [1140 1146];
sc.handTakeTool = zeros(0,2);                 % palm reaches the tool held by the robot (used by trajToolFetchHRC)
prm.scenario = sc;

% ------------------------------------------------------------ live workstation calibration
prm.calibrationFile = fullfile(fileparts(mfilename('fullpath')), 'workspace_calibration.json');
prm.calibrated = false;
prm.assemblyArea = struct('x',[0.65 0.95],'y',[-0.05 0.25],'height',0.25);   % default (no calibration)
if exist(prm.calibrationFile, 'file') == 2
    prm = applyWorkspaceCalibration(prm);
end
cache = prm;
end

% =========================================================================
function prm = applyWorkspaceCalibration(prm)
% Places and safety distances from workspace_calibration.py (frame W: origin
% robot base on the table, x towards the worker, y his right, z up, metres).
%   robot poses : safe position (idle / rest / stopped), tool storage, handover
%                 point (assistive), collaborative work point (inside the
%                 assembly area, on the robot side)
%   assembly area: fixed rectangle on the table (prm.assemblyArea.x, .y, .height)
%   dCollabMax  : a hand anywhere in the assembly area is in the collaborative
%                 zone while the robot works at the collaborative point
%   dAssistMax  : a hand anywhere in the assembly area is in the assistive zone
%                 while the robot holds the tool at the handover point
%   dProhibit   : from the workstation file (zones.dProhibit = config prohibit_radius_m, 0.05 m); default above.
% The calibration program already checked that the robot at the safe position
% is clear of the assembly area and that the robot paths do not cross it.
% ILLUSTRATIVE VALUES FOR THE VIRTUAL-ROBOT TESTS, NOT VALIDATED SAFETY DISTANCES.
try
    c = jsondecode(fileread(prm.calibrationFile));
    if ~isfield(c,'robot_poses_W') || ~isfield(c,'places_W')
        error('old calibration format - run workspace_calibration.py again');
    end
    rp = c.robot_poses_W;
    g = prm.geo;
    g.poseSolo    = rp.safe(:)';
    g.poseStorage = rp.storage(:)';
    g.poseAssist  = rp.handover(:)';
    g.poseCollab  = rp.collab(:)';
    if isfield(c,'robot') && isfield(c.robot,'shoulder_height')
        g.robotShoulder = [0 0 c.robot.shoulder_height];   % virtual robot as high as the robot object
    end
    a = c.places_W.assembly_area;
    prm.assemblyArea = struct('x',a.x(:)','y',a.y(:)','height',a.height);
    g.palmWork = [mean(a.x) mean(a.y) 0.10];       % hand at the centre of the assembly area
    g.entryPose = {g.poseSolo, g.poseSolo, g.poseCollab};
    prm.dCollabMax = c.zones.dCollabMax;
    prm.dAssistMax = c.zones.dAssistMax;
    if isfield(c.zones,'dProhibit')                % prohibited distance (config prohibit_radius_m)
        prm.dProhibit = c.zones.dProhibit;
    end
    if prm.assistZone.useWithCalibration
        prm.assistZone.enabled = true;
        prm.assistZone.centre = g.poseAssist;     % handover point
        if isfield(c.zones,'assistRadius')        % radius from the workstation file (config assist_radius_m)
            prm.assistZone.radius = c.zones.assistRadius;
        end
    end
    prm.geo = g;
    prm.calibrated = true;
    prm.calibration = c;
catch err
    warning('stateDefinitionHRC:calibration', 'Cannot use %s (%s): default geometry used', ...
        prm.calibrationFile, err.message);
end
end
