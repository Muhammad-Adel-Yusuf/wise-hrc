function geo = geometryObservationHRC(raw, tNow, prm)
% Observed safety geometry of one cycle. Used by BOTH observation sources
% (syntheticObservationHRC and realCameraObservationAdapter), so the zone,
% risk and verification rules are identical for synthetic and real data.
%
% raw (measurement contract, frame W, meters, seconds):
%   raw.timestamp          time at which the geometry was measured [s]
%   raw.humanInCell        1 person in the work cell, 0 verified empty, NaN unknown
%   raw.handTracked        1 if the palm reference is tracked
%   raw.limbTracked        1 if shoulder, elbow, wrist and palm are tracked
%   raw.human.shoulder / elbow / wrist / palm      1x3 points (NaN if not tracked)
%   raw.robot.base / shoulder / elbow / tool       1x3 MEASURED robot points
%   raw.robotPlanned.base / shoulder / elbow / tool  robot points at the end of
%                          the planned path horizon (prm.pathHorizon)
%   raw.pathValid          1 if the planned path is available
%   raw.measuredRobotSpeed measured tool speed [m/s] (encoders), not the command
%   raw.commandedMotion    commanded motion level 0/1/2 (for information)
%   raw.safetyConfidence   0..1 confidence of the geometry sensing
%   raw.externalHazard     1 if another hazard signal is active (light curtain, ...)
%
% geo (observation, named fields): everything in raw plus
%   geo.age, geo.fresh, geo.handOK, geo.limbOK, geo.pathOK, geo.verified,
%   geo.unverifiedReason, geo.handInteractionDistance, geo.minUpperLimbClearance,
%   geo.closestHuman, geo.closestRobot, geo.armClearance (upper arm + forearm
%   only, used for the stationary handover), geo.pathClearance, geo.pathConflict,
%   geo.zone (Z), geo.risk (R), geo.separationVerified, geo.frame, geo.units
% Missing, stale or low-confidence geometry is NEVER read as separation:
% zone / risk become NaN and verified = 0.
geo = raw;
geo.frame = 'W: origin robot base, x towards the human, z up';
geo.units = 'm, s';
geo.age = tNow - raw.timestamp;
geo.fresh = ~isnan(geo.age) && geo.age <= prm.maxGeometryAge + 1e-9;
confOK = raw.safetyConfidence >= prm.minSafetyConfidence;
h = raw.human;
handPts = all(isfinite(h.palm));
% PALM-ONLY (prm.palmOnlyClearance = true, set in liveHRC): the palm is the
% only point used for safety, so a tracked palm is a tracked "limb" and the
% clearance is palm sphere -> robot capsules (geometryHRC). Shoulder, elbow
% and wrist are not required and not checked.
palmOnly = isfield(prm,'palmOnlyClearance') && prm.palmOnlyClearance;
if palmOnly
    limbPts = handPts;
    limbTrk = raw.handTracked==1;
else
    limbPts = all(isfinite([h.shoulder h.elbow h.wrist h.palm]));
    limbTrk = raw.limbTracked==1;
end
empty = raw.humanInCell==0;                     % verified empty work cell
geo.handOK = geo.fresh && confOK && (empty || (raw.handTracked==1 && handPts));
geo.limbOK = geo.fresh && confOK && (empty || (limbTrk && limbPts));
geo.pathOK = geo.fresh && raw.pathValid==1;
geo.verified = geo.handOK && geo.limbOK && geo.pathOK;
if ~geo.fresh
    geo.unverifiedReason = 'stale_geometry';
elseif ~confOK
    geo.unverifiedReason = 'low_safety_confidence';
elseif ~geo.handOK
    geo.unverifiedReason = 'hand_not_tracked';
elseif ~geo.limbOK
    geo.unverifiedReason = 'upper_limb_not_tracked';
elseif ~geo.pathOK
    geo.unverifiedReason = 'path_not_available';
else
    geo.unverifiedReason = '';
end
% ---------------- distances
geo.handInteractionDistance = NaN;
geo.minUpperLimbClearance = NaN;
geo.closestHuman = ''; geo.closestRobot = '';
geo.armClearance = NaN;     % clearance of upper arm and forearm only (hand excluded)
geo.pathClearance = NaN;
if empty && geo.handOK && geo.limbOK
    geo.handInteractionDistance = Inf;
    geo.minUpperLimbClearance = Inf;
    geo.armClearance = Inf;
    geo.pathClearance = Inf;
elseif geo.limbOK
    m = geometryHRC(h, raw.robot, prm);
    geo.handInteractionDistance = m.handInteractionDistance;
    geo.minUpperLimbClearance = m.minUpperLimbClearance;
    geo.closestHuman = m.closestHuman; geo.closestRobot = m.closestRobot;
    geo.armClearance = m.armClearance;
    if geo.pathOK
        mp = geometryHRC(h, raw.robotPlanned, prm);
        geo.pathClearance = min(mp.minUpperLimbClearance, m.minUpperLimbClearance);
    end
elseif geo.handOK
    geo.handInteractionDistance = norm(h.palm(:) - raw.robot.tool(:));
end
% ---------------- path conflict: the planned motion would reach the limb
if geo.pathOK && ~isnan(geo.pathClearance)
    geo.pathConflict = double(raw.commandedMotion > 0 && geo.pathClearance < prm.cActive);
else
    geo.pathConflict = NaN;
end
% ---------------- zone and risk
if geo.handOK
    geo.zone = zoneClassifyHRC(geo.handInteractionDistance, prm, geo.human.palm);
else
    geo.zone = NaN;
end
geo.risk = riskEstimateHRC(geo, prm);
% whole-arm separation is verified by the clearance of the whole limb and
% by the path check, never by the palm zone alone. The clearance (>= cSeparated
% 0.30 m) replaces "risk 0": risk 1 also means a palm inside the collaborative
% (0.35 m) or assistive zone, which says nothing about a robot working apart
% (2026-10-08: normal solo speed down to 0.30 m for a rested worker).
geo.separationVerified = geo.verified && ~isnan(geo.risk) && geo.risk~=2 && geo.pathConflict==0 ...
    && geo.minUpperLimbClearance >= prm.cSeparated && raw.externalHazard==0;
