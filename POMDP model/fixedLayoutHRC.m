function prm = fixedLayoutHRC(prm)
% FIXED WORKSTATION LAYOUT: the places and zones are taken from the values
% below instead of the ones saved by workspace_calibration.py, so they stay
% the same in every experiment. Called at the end of stateDefinitionHRC.
%
% The camera program (main.py) still needs workspace_calibration.json for
% the workstation frame W (origin = foot of the bottle, x towards the
% assembly area): the worker's arms are measured in that frame. Keep the
% camera, the table and the bottle where they were when that file was made.
% Set use = false to go back to the calibrated places.
%
% Frame W, metres: x towards the worker, y to his right, z up (table = 0).
use = true;
if ~use, return; end
L.storageXY   = [0.10  0.03];     % tool storage (tissue box)
L.handoverXY  = [0.175 0.215];    % handover point = centre of the assistive zone
L.areaX       = [0.34  0.79];     % assembly area (rectangle on the table)
L.areaY       = [-0.11 0.135];
L.areaHeight  = 0.25;
L.dCollabMax  = 0.42;             % collaborative distance from the robot tool
L.assistRadius = 0.15;            % assistive zone: circle around the handover point
% heights [m]: from the calibration file when it exists, else these values
L.robotHeight    = 0.22;          % safe position = bottle tip
L.storageZ       = 0.11;
L.handoverZ      = 0.20;
L.collabZ        = 0.16;
g = prm.geo;
if prm.calibrated
    L.robotHeight = g.poseSolo(3);
    L.storageZ = g.poseStorage(3);
    L.handoverZ = g.poseAssist(3);
    L.collabZ = g.poseCollab(3);
end
g.poseSolo    = [0 0 L.robotHeight];
g.poseStorage = [L.storageXY L.storageZ];
g.poseAssist  = [L.handoverXY L.handoverZ];
g.poseCollab  = [L.areaX(1) + 0.05, mean(L.areaY), L.collabZ];   % inside the area, robot side
g.robotShoulder = [0 0 L.robotHeight];
g.palmWork = [mean(L.areaX) mean(L.areaY) 0.10];
g.entryPose = {g.poseSolo, g.poseSolo, g.poseCollab};
prm.geo = g;
prm.assemblyArea = struct('x',L.areaX,'y',L.areaY,'height',L.areaHeight);
prm.dCollabMax = L.dCollabMax;
prm.assistZone.enabled = true;
prm.assistZone.centre = g.poseAssist;
prm.assistZone.radius = L.assistRadius;
prm.fixedLayout = L;
end
