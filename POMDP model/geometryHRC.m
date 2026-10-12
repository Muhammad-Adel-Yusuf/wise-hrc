function m = geometryHRC(human, robot, prm)
% Two separate geometric measures (frame W, meters, see stateDefinitionHRC)
%
% human : struct with 1x3 points shoulder, elbow, wrist, palm (right upper limb)
% robot : struct with 1x3 points base, shoulder, elbow, tool
%
% m.handInteractionDistance
%     distance between the designated hand reference (palm point) and the
%     designated robot interaction reference (tool / handover point).
%     Used ONLY for the hand interaction zone Z (zoneClassifyHRC).
% m.minUpperLimbClearance
%     minimum surface-to-surface clearance between the upper-limb capsules
%       upper arm (shoulder-elbow), forearm (elbow-wrist), hand (wrist-palm)
%     and the robot capsules
%       column (base-shoulder), upper link (shoulder-elbow),
%       forearm link + tool (elbow-tool)
%     = segment distance - radius_human - radius_robot (negative = overlap).
%     Used for collision risk (riskEstimateHRC). It includes the hand, so
%     the palm-to-tool pair is checked twice: as a zone and as clearance.
% m.clearance(i,j), m.closestHuman, m.closestRobot : details of all 9 pairs
%
% Approximation: capsules (line segments with a radius) are a coarse
% envelope of the limb and the robot. They ignore the fingers, the shape
% of the tool, the torso and the head, and their radii are illustrative.
% The robot elbow point is a kinematic placeholder, not a robot model.
g = prm.geo;
hSeg = {human.shoulder, human.elbow; human.elbow, human.wrist; human.wrist, human.palm};
hRad = [g.rUpperArm g.rForearm g.rHand];
hName = {'upperArm','forearm','hand'};
rSeg = {robot.base, robot.shoulder; robot.shoulder, robot.elbow; robot.elbow, robot.tool};
rRad = [g.rColumn g.rUpperLink g.rForearmTool];
rName = {'column','upperLink','forearmLinkTool'};
m.handInteractionDistance = norm(human.palm(:) - robot.tool(:));
% PALM-ONLY CLEARANCE (prm.palmOnlyClearance = true, set in liveHRC): the
% clearance is measured from the palm only (sphere of radius rHand) to the
% robot capsules; upper arm and forearm are ignored (m.armClearance = Inf).
palmOnly = isfield(prm,'palmOnlyClearance') && prm.palmOnlyClearance;
if palmOnly
    hSeg = {human.palm, human.palm};
    hRad = g.rHand;
    hName = {'palm'};
end
nh = size(hSeg,1);
m.clearance = zeros(nh,3);
for i = 1:nh
    for j = 1:3
        m.clearance(i,j) = segmentDistanceHRC(hSeg{i,1},hSeg{i,2},rSeg{j,1},rSeg{j,2}) - hRad(i) - rRad(j);
    end
end
[m.minUpperLimbClearance, k] = min(m.clearance(:));
[i, j] = ind2sub([nh 3], k);
m.closestHuman = hName{i};
m.closestRobot = rName{j};
if palmOnly
    m.armClearance = Inf;                        % upper arm / forearm not checked
else
    m.armClearance = min(min(m.clearance(1:2,:)));
end
