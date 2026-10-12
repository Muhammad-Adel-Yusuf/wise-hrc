function Z = zoneClassifyHRC(d, prm, palm)
% Hand interaction zone from the palm-to-interaction-reference distance d [m]
% (ILLUSTRATIVE boundaries, not validated safety distances)
%   d <  prm.dProhibit            -> Z = 0  hand violates the prohibited boundary
%   d <= prm.dCollabMax           -> Z = 1  collaborative envelope
%   d <= prm.dAssistMax           -> Z = 2  assistive region
%   otherwise                     -> Z = 3  separated region
% The rules are checked in this order, so the regions never overlap:
% precedence 0 > 1 > 2 > 3, and a boundary value belongs to the zone closer
% to the robot (except d == dProhibit, which is already collaborative).
% d = NaN (hand not tracked or not verified) -> Z = NaN (unknown).
% FIXED ASSISTIVE ZONE (prm.assistZone.enabled, live tests): the assistive
% region is NOT a distance from the robot but a fixed circle (top view) of
% radius prm.assistZone.radius around the handover point; zones 0 and 1 stay
% distances to the robot tool:
%   d <  dProhibit -> 0,  d <= dCollabMax -> 1,
%   palm within radius of the handover point (horizontal) -> 2,  otherwise 3.
% Z describes the PALM only. Elbow, forearm and upper arm are handled by
% the clearance in riskEstimateHRC.
if isnan(d)
    Z = NaN;
elseif d < prm.dProhibit
    Z = 0;
elseif d <= prm.dCollabMax
    Z = 1;
elseif nargin >= 3 && isfield(prm,'assistZone') && prm.assistZone.enabled
    if norm(palm(1:2) - prm.assistZone.centre(1:2)) <= prm.assistZone.radius
        Z = 2;
    else
        Z = 3;
    end
elseif d <= prm.dAssistMax
    Z = 2;
else
    Z = 3;
end
