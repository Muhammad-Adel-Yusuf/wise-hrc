function R = riskEstimateHRC(geo, prm)
% Interaction risk from the observed geometry (ILLUSTRATIVE thresholds)
% geo fields used: verified, zone, minUpperLimbClearance, measuredRobotSpeed,
%                  pathConflict, externalHazard
%   R = NaN  safety information not verified (missing, stale, low confidence)
%   R = 2    active:    hand in the prohibited region (Z = 0)
%                       OR whole-limb clearance < cActive + speed*tStop
%                       OR planned robot path conflicts with the limb
%                       OR an external hazard signal is set
%   R = 1    potential: clearance < cWarn OR palm in the collaborative /
%                       assistive band (Z = 1 or 2)
%   R = 0    otherwise
% The clearance covers hand, forearm, elbow and upper arm, so a palm that
% is correctly placed (Z = 1) does not hide an elbow or forearm hazard, and
% a separated palm (Z = 3) does not hide a forearm close to a robot link.
if ~geo.verified
    R = NaN;
    return
end
c = geo.minUpperLimbClearance;
sp = prm.cActive + geo.measuredRobotSpeed*prm.tStop;     % speed-dependent protective distance
if geo.zone==0 || c < sp || geo.pathConflict==1 || geo.externalHazard==1
    R = 2;
elseif c < prm.cWarn || geo.zone==1 || geo.zone==2
    R = 1;
else
    R = 0;
end
