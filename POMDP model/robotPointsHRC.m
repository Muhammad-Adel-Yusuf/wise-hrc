function r = robotPointsHRC(tool, prm)
% Synthetic robot link points for a tool position (frame W, meters)
% base and shoulder are fixed; the robot elbow is a kinematic placeholder
% (midpoint of shoulder and tool lifted by prm.geo.robotElbowLift), not a
% model of a specific robot.
g = prm.geo;
el = 0.5*(g.robotShoulder + tool) + [0 0 g.robotElbowLift];
r = struct('base', g.robotBase, 'shoulder', g.robotShoulder, 'elbow', el, 'tool', tool);
