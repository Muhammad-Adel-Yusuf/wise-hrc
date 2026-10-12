function [aF,sup,info] = safetySupervisorHRC(aN,O,geo,M,sup)
% Safety supervisor. It uses only the CURRENT observation (O, geo) and its
% own timers. It never reads the belief, and the nominal action aN is only
% a candidate: the supervisor decides the final action aF.
%
% aN  : nominal action of the QMDP planner (1..10), or 0 = task-layer transfer
% O   : observation vector (stateDefinitionHRC), used for attention, presence,
%       human-state confidence and the emergency stop
% geo : observed safety geometry (geometryObservationHRC)
% M   : task mode from the task layer
% sup : supervisor memory ([] at the start), fields
%       distraction  distraction duration [s]
%       faceLoss     continuous face-detection loss [s]
%       tau          risky-inattention duration [s] (distraction while R ~= 0)
%       latched      1 while a protective stop is active
%       clearCount   consecutive verified all-clear epochs since the hazard
%       cause        reason of the active stop
%
% PROTECTIVE STOP (10) when any of these holds (checked in this order):
%   emergency stop; safety geometry not verified (missing, stale, low safety
%   confidence, hand or limb not tracked, path unknown); active risk R = 2
%   (includes elbow/forearm/upper-arm clearance and path conflict); hand in
%   the prohibited region Z = 0; path conflict; distracted >= 2 s while R ~= 0.
% STATIONARY HANDOVER (O(16) = 1, assistive mode): while the robot holds the
%   tool at the handover point and is completely still, the hand may come
%   inside the prohibited distance to take the tool. The robot keeps holding
%   (action 4, stopped) instead of a protective stop; any upper-arm/forearm
%   hazard, robot motion, path conflict or external hazard still gives a
%   protective stop. Motion is allowed again only after the hand is clear
%   (same verified release rule as a protective stop).
% RELEASE of a stop only after prm.releaseSteps consecutive epochs in which
%   the hazard is observed to be gone with verified, high-confidence
%   geometry (hysteresis: hand distance >= handRelease, clearance >= cRelease).
%   A motion request of the planner never releases a stop.
% MODE RULES when no stop is needed:
%   solo          normal speed only when whole-arm separation is verified
%                 (geo.separationVerified: clearance >= prm.cSeparated = 0.30 m,
%                 NOT the palm zone); otherwise slow (2). Fatigue, distraction
%                 or low human-state confidence alone never stop it here; with
%                 medium / high fatigue soloFatigueSpeedHRC (after this
%                 supervisor) slows the robot further below 0.30 m and stops
%                 it near the active-risk distance.
%   assistive     approach, assistance and retreat under full monitoring
%   collaborative 5, 6 and 8 only with the palm in the envelope (Z = 1);
%                 otherwise the stopped wait 7 (the robot never chases the hand)
%   low human-state confidence (O(7)) in assistive/collaborative mode:
%                 collaborative normal (5) is reduced to slow and smooth (6)
prm = stateDefinitionHRC();
act = actionDefinitionHRC();
if isempty(sup)
    sup = struct('distraction',0,'faceLoss',0,'tau',0,'latched',0,'clearCount',0,'cause','');
end
dt = prm.dt;
H = O(4); A = O(3); face = O(8); hConf = O(7);
if isnan(H), H = 1; end
if isnan(hConf), hConf = 0; end
risk = geo.risk; zone = geo.zone;
info = struct('overridden',0,'reason','','stopRequested',0,'released',0, ...
    'distracted',0,'cleared',0);
% ------------------------------------------------ attention timers (unchanged rule)
faceOk = (face==1) && ~isnan(A);
if H==0
    sup.distraction = 0; sup.faceLoss = 0;
    distracted = 0;
else
    if faceOk
        sup.faceLoss = 0;
    else
        sup.faceLoss = sup.faceLoss + dt;         % face lost
    end
    distracted = (faceOk && A==2) || sup.faceLoss >= prm.faceLossGrace;
    if distracted
        sup.distraction = sup.distraction + dt;
    elseif faceOk && A==0
        sup.distraction = 0;                      % focused again
    end                                           % uncertain / short face loss: timer kept
end
riskKnown = ~isnan(risk);
sup.tau = sup.distraction*(~riskKnown || risk~=0);
info.distracted = distracted;
% ------------------------------------------------ stationary handover
% The hand may come inside the prohibited distance of the tool ONLY while
% the robot holds the tool at the handover point and is completely still
% (commanded stopped and measured speed ~ 0), and only the hand is close:
% upper arm and forearm must keep the normal clearance.
handoverActive = numel(O) >= 16 && O(16)==1;
stationary = geo.commandedMotion==0 && geo.measuredRobotSpeed <= prm.stationarySpeed;
handoverContact = handoverActive && M==1 && stationary && geo.verified ...
    && (zone==0 || risk==2) && geo.armClearance >= prm.cActive ...
    && geo.externalHazard==0 && geo.pathConflict~=1;
% ------------------------------------------------ stop conditions
reason = '';
if O(10)~=0
    reason = 'emergency_stop';
elseif ~geo.verified
    reason = ['safety_unverified:' geo.unverifiedReason];
elseif handoverContact
    aF = 4;                                     % stay still and hold the tool
    sup.latched = 1; sup.clearCount = 0; sup.cause = 'handover_contact';
    info.reason = 'handover_contact_hold_stationary';
    info.overridden = (aF~=aN);
    return
elseif risk==2
    reason = 'active_interaction_risk';
elseif zone==0
    reason = 'hand_prohibited_zone';
elseif geo.pathConflict==1
    reason = 'path_conflict';
elseif distracted && sup.distraction >= prm.distractionLimit && risk~=0
    reason = 'risky_distraction';
end
if ~isempty(reason)
    aF = 10;
    sup.latched = 1; sup.clearCount = 0; sup.cause = reason;
    info.stopRequested = 1;
    info.reason = reason;
    info.overridden = (aF~=aN);
    return
end
% ------------------------------------------------ release of an active stop
if sup.latched==1
    cleared = geo.verified && O(10)==0 && risk~=2 && zone~=0 && geo.pathConflict==0 ...
        && geo.handInteractionDistance >= prm.handRelease ...
        && geo.minUpperLimbClearance >= prm.cRelease ...
        && geo.safetyConfidence >= prm.minReleaseConfidence;
    info.cleared = cleared;
    if cleared
        sup.clearCount = sup.clearCount + 1;
    else
        sup.clearCount = 0;
    end
    if sup.clearCount < prm.releaseSteps
        if strcmp(sup.cause,'handover_contact')
            aF = 4;                             % handover: keep holding still
            info.reason = 'handover_hold_until_hand_clear';
        else
            aF = 10;
            info.reason = 'stop_held_until_verified_clearance';
        end
        info.overridden = (aF~=aN);
        return
    end
    sup.latched = 0; sup.clearCount = 0; sup.cause = '';
    info.released = 1;
end
% ------------------------------------------------ transfer phase of the task layer
if aN==0
    aF = 0;
    info.reason = 'transfer_monitored';
    return
end
% ------------------------------------------------ admissible action
aF = aN;
mask = actionMaskHRC(M, zone);
if mask(aF)==0
    if M==2 && any(aN==act.collabExecution) && zone~=1
        info.reason = 'hand_outside_envelope_wait';
    else
        info.reason = 'action_not_allowed';
    end
    cand = [prm.waitAction(M+1) 9 10];
    aF = cand(find(mask(cand),1));
end
if M==0
    if geo.separationVerified
        if aF==10
            aF = 1;                               % no safety reason to stop
            info.reason = 'solo_separation_verified_continue';
        end
    elseif aF==1 || aF==8
        aF = 2;                                   % palm may be far, but the limb is not verified separated
        info.reason = 'solo_separation_not_verified_slow';
    end
end
if M~=0 && hConf < prm.minHumanConfidence && aF==5
    aF = 6;
    info.reason = 'low_human_state_confidence_slow';
end
info.overridden = (aF~=aN);
