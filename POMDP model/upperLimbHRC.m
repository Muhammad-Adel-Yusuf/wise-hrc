function h = upperLimbHRC(palm, lean, raised, prm)
% Synthetic right upper limb from the palm position (frame W, meters)
% palm   : 1x3 palm reference point
% lean   : 1 = shoulder leaning towards the robot (prm.geo.humanShoulderLean)
% raised : 1 = elbow raised towards the robot (pole prm.geo.elbowPoleRaised)
% h      : struct with shoulder, elbow, wrist, palm (1x3 each)
% wrist = palm moved palmToWrist towards the shoulder; elbow from a 2-link
% inverse kinematics (upper arm and forearm lengths, pole direction).
% This is a geometric approximation for synthetic data, not a
% biomechanical model (no joint limits, no torso, fixed segment lengths).
g = prm.geo;
if lean, S = g.humanShoulderLean; else, S = g.humanShoulder; end
if raised, pole = g.elbowPoleRaised; else, pole = g.elbowPoleDown; end
P = palm;
W = P + g.palmToWrist*(S - P)/norm(S - P);
Lu = g.upperArmLength; Lf = g.forearmLength;
d = norm(W - S);
d = min(max(d, abs(Lu-Lf) + 1e-3), Lu + Lf - 1e-3);
u = (W - S)/norm(W - S);
a = (Lu^2 - Lf^2 + d^2)/(2*d);
hh = sqrt(max(Lu^2 - a^2, 0));
n = pole - (pole*u')*u;
n = n/norm(n);
E = S + a*u + hh*n;
h = struct('shoulder', S, 'elbow', E, 'wrist', W, 'palm', P);
