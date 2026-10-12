function env = syntheticEnvironmentHRC(env, prm, k, M, a)
% Synthetic work cell (SYNTHETIC DEMONSTRATION, illustrative geometry)
%
% env = syntheticEnvironmentHRC([], prm)          initial work cell
% env = syntheticEnvironmentHRC(env, prm, k, M, a) advance one epoch: the
%       robot executes action a (0 = task-layer transfer) during epoch k
%       in task mode M, and the human moves according to the scenario.
%
% Truth produced (frame W, meters):
%   env.human.shoulder/elbow/wrist/palm, env.robot.base/shoulder/elbow/tool,
%   env.robotPlanned (pose at the end of the planned path horizon),
%   env.measuredSpeed (actual tool speed), env.commandedMotion,
%   env.raw (noise-free, fully valid measurement), env.truth (geometry
%   observation computed from env.raw, used only for logging / ground truth)
%
% Robot motion: the tool moves towards a target pose with the commanded
% speed (prm.geo.robotSpeed). It never jumps. Targets:
%   task layer (a = 0) -> env.taskTarget if set (e.g. tool storage, safe
%                         position during a rest), otherwise the entry pose of
%                         the mode (transfer); once the target is reached the
%                         robot waits there with commanded motion 0
% Optional start pose: prm.geo.poseStart (default: solo / safe position)
%   solo             -> solo pose
%   approach (3), assist (8) -> assistive handover point;  retreat (9) -> solo pose
%   collaborative    -> fixed collaborative work point (never follows the hand)
%   hold / stop (4, 7, 10) -> stays where it is (speed 0)
% Braking distance is ignored (the robot stops within the epoch).
%
% Human motion is driven only by the scenario (prm.scenario), never by the
% robot action: a protective stop does not make the hand withdraw.
%   handRest    palm moves to the rest position
%   handReach   palm reaches next to the robot base (palm separated from the
%               tool, forearm close to the robot column)
%   handIntrude palm moves inside the prohibited distance of the tool and
%               STAYS there for the whole window; the end of the window is
%               the explicit withdrawal event
%   handTakeTool palm moves to the tool held by the robot (grasp)
%   elbowRaise  shoulder leans and the elbow is raised towards the robot
%               while the palm stays at its working position
% Limb points: upperLimbHRC, robot points: robotPointsHRC.
g = prm.geo;
if isempty(env)
    env.k = 0;
    env.t = 0;
    if isfield(g,'poseStart') && ~isempty(g.poseStart)
        env.tool = g.poseStart;                   % optional start pose (e.g. fatigueRunHRC)
    else
        env.tool = g.poseSolo;
    end
    env.palm = g.palmWork;
    env.target = env.tool;
    env.intrudeTarget = [];
    env.lean = 0;
    env.raised = 0;
    env.measuredSpeed = 0;
    env.commandedMotion = 0;
    env.oscillate = 0;
    env.taskTarget = [];
    env = updatePoints(env, prm);
    return
end
act = actionDefinitionHRC();
sc = prm.scenario;
% ---------------- robot
if a==0
    motion = 1;                                   % task-layer motion at slow speed
    if isfield(env,'taskTarget') && ~isempty(env.taskTarget)
        target = env.taskTarget;                  % e.g. tool storage (trajToolFetchHRC)
    else
        target = g.entryPose{M+1};                % transfer to the entry pose of the mode
    end
    if norm(target - env.tool) < 1e-6
        motion = 0;                               % target reached: the robot waits there, stopped
    end
    osc = 0;
else
    motion = act.motion(a);
    osc = any(a==[1 2 5 6 8]);                    % small working motion
    if any(a==[4 7 10])
        target = env.tool;
    elseif M==0
        target = g.poseSolo;
    elseif M==1
        if a==3 || a==8
            target = g.poseAssist;                % approach / assist at the handover point
        elseif a==9
            target = g.poseSolo;
        else
            target = env.target;                  % keep the current working pose
        end
    else
        target = g.poseCollab;
    end
end
if osc
    goal = target + g.workAmplitude*[0 sin(0.6*k) 0];
else
    goal = target;
end
speed = g.robotSpeed(motion+1);
if isfield(env,'speedScale'), speed = speed*env.speedScale; end   % solo fatigue / distance factor
toolOld = env.tool;
dvec = goal - env.tool;
dist = norm(dvec);
if dist > 0 && speed > 0
    env.tool = env.tool + dvec/dist*min(dist, speed*prm.dt);
end
env.target = target;
env.measuredSpeed = norm(env.tool - toolOld)/prm.dt;
env.commandedMotion = motion;
% ---------------- human (scenario only)
palmGoal = g.palmWork;
if inWin(k, sc.handRest),  palmGoal = g.palmRest;  end
if inWin(k, sc.handReach), palmGoal = g.palmReach; end
if isfield(sc,'handTakeTool') && inWin(k, sc.handTakeTool)
    u = g.palmWork - env.tool; u = u/norm(u);
    palmGoal = env.tool + g.graspDistance*u;      % palm grasps the tool held by the robot
end
if inWin(k, sc.handIntrude)
    if isempty(env.intrudeTarget)
        % entry into the prohibited region, fixed for the whole window
        u = g.palmWork - env.tool; u = u/norm(u);
        env.intrudeTarget = env.tool + g.intrudeDistance*u;
    end
    palmGoal = env.intrudeTarget;
else
    env.intrudeTarget = [];                       % withdrawal: end of the window
end
dvec = palmGoal - env.palm;
dist = norm(dvec);
if dist > 0
    env.palm = env.palm + dvec/dist*min(dist, g.handSpeed*prm.dt);
end
env.lean = inWin(k, sc.elbowRaise);
env.raised = env.lean;
env.k = k;
env.t = k*prm.dt;
env.oscillate = osc;
env = updatePoints(env, prm);
end

% =========================================================================
function env = updatePoints(env, prm)
g = prm.geo;
env.human = upperLimbHRC(env.palm, env.lean, env.raised, prm);
env.robot = robotPointsHRC(env.tool, prm);
% planned pose after the path horizon along the current commanded motion
d = env.target - env.tool;
v = g.robotSpeed(env.commandedMotion+1);
if norm(d) > 0 && v > 0
    planned = env.tool + d/norm(d)*min(norm(d), v*prm.pathHorizon);
else
    planned = env.tool;
end
env.robotPlanned = robotPointsHRC(planned, prm);
raw.timestamp = env.t;
raw.humanInCell = 1;
raw.handTracked = 1;
raw.limbTracked = 1;
raw.human = env.human;
raw.robot = env.robot;
raw.robotPlanned = env.robotPlanned;
raw.pathValid = 1;
raw.measuredRobotSpeed = env.measuredSpeed;
raw.commandedMotion = env.commandedMotion;
raw.safetyConfidence = 1;
raw.externalHazard = 0;
env.raw = raw;
env.truth = geometryObservationHRC(raw, env.t, prm);
end

function tf = inWin(k, W)
tf = ~isempty(W) && any(k >= W(:,1) & k <= W(:,2));
end
