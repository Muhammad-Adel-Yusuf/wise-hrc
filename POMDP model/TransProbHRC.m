function T = TransProbHRC(S, ev, tot)
% T = TransProbHRC()           table of all states (34992 x 130)
% T = TransProbHRC(S, ev, tot) row of one state S with observed events ev and time on task tot
%
% PLANNER MODEL (predicted geometry). For every action a there are 13
% successor outcomes, column 13*(a-1)+j (successor states in NextStateHRC):
%   j = 1  nominal                         j = 8  attention gets worse
%   j = 2  instruction +1                  j = 9  attention gets better
%   j = 3  instruction -1 (backtracking)   j = 10 hand crosses the prohibited boundary
%   j = 4  physical fatigue +1             j = 11 robot motion changes the hand band
%   j = 5  physical fatigue -1                    (approach -> assistive, retreat -> separated)
%   j = 6  mental fatigue +1               j = 12 environmental change by the HUMAN:
%   j = 7  mental fatigue -1                      withdrawal from an unsafe condition,
%                                                 hand leaving / returning to the envelope,
%                                                 decay of a residual potential risk
%                                          j = 13 elbow/forearm/upper-arm hazard onset
% Fatigue and attention transitions are unchanged by this update. Fatigue
% changes by at most one level per epoch. Solo task with the hand separated:
% the robot action has no effect on the human transition.
% Geometry rules:
%  - the protective stop (10) does NOT change the hand zone and does NOT
%    make withdrawal more likely: pWithdraw is the same for every action
%  - approach / retreat change the band only with probability pApproach /
%    pRetreat per epoch (the robot travels, it does not jump)
%  - no collaborative action changes the hand band
prm = stateDefinitionHRC();
act = actionDefinitionHRC();
if nargin > 0
    if nargin < 2 || isempty(ev), ev = prm.evExpected; end
    if nargin < 3 || isempty(tot), tot = prm.timePlan; end
    T = transRow(S, ev, tot, prm, act);
    return
end
T = zeros(prm.nS,10*prm.nOut);
k = 1;
for M = 0:2
    for I = 1:6
        for H = 0:1
            for Fp = 0:2
                for Fm = 0:2
                    for A = 0:2
                        for Z = 0:3
                            for V = 0:2
                                for R = 0:2
                                    T(k,:) = transRow([M I H Fp Fm A Z V R], prm.evExpected, prm.timePlan, prm, act);
                                    k = k+1;
                                end
                            end
                        end
                    end
                end
            end
        end
    end
end
end

function t = transRow(S, ev, tot, prm, act)
M = S(1); I = S(2); H = S(3); Fp = S(4); Fm = S(5); A = S(6); Z = S(7); R = S(9);
nO = prm.nOut;
ev(isnan(ev)) = 0;
t = zeros(1,10*nO);
coupled = ~(M==0 && Z==3);          % robot can affect the human
hazard = (Z==0) || (R==2);          % unresolved unsafe condition
for a = 1:10
    p = zeros(1,nO);
    Vn = act.motion(a);
    isPause = coupled && any(a==act.pause);
    confirm = coupled && any(a==act.confirm);
    if H==1
        % ---------------- human state (unchanged model)
        pLoad = prm.physBase + prm.physTime*tot + coupled*prm.robotPhys(a);
        pLoad = min(max(pLoad,0),1);
        robotM = coupled*prm.robotMent(a) + (M==0 && coupled && Vn>0)*prm.soloNear;
        mLoad = prm.mentBase + prm.kDiff*prm.difficulty(I) + prm.kBack*ev(1) + prm.kView*ev(2) ...
            + prm.kHes*ev(3) + prm.kCorr*ev(4) + prm.mentTime*tot + robotM;
        mLoad = min(max(mLoad,0),1);
        rest = 1 + (prm.restFactor-1)*isPause;
        p(4) = prm.physUp*pLoad*(Fp<2);
        p(5) = prm.physDown*(1-pLoad)*rest*(Fp>0);
        p(6) = prm.mentUp*mLoad*(Fm<2);
        p(7) = prm.mentDown*(1-mLoad)*rest*(Fm>0);
        worse = prm.attWorse(A+1);
        better = prm.attBetter(A+1);
        if confirm
            worse = worse*(1-prm.attConfirm);
            better = better + prm.attConfirm*(A>0);
        end
        p(8) = worse*(A<2);
        p(9) = better*(A>0);
        adv = prm.instrAdvance*(1-prm.advFatigueLoss*Fm/2);
        if A==2, adv = adv*(1-prm.advDistractLoss); end
        if coupled && a==8 && M~=0, adv = adv*(1+prm.assistGain); end
        if isPause, adv = adv*prm.holdFactor; end
        p(2) = adv;
        p(3) = (I>1)*(prm.instrBack + prm.backFatigue*Fm/2);
        % ---------------- predicted geometry: human-caused hazards
        if Z==1 || Z==2
            if Vn>0
                p(10) = prm.intrusion(A+1);
            else
                p(10) = prm.intrusionStopped;
            end
            if R~=2 && Vn>0
                p(13) = prm.limbHazard(A+1);
            end
        elseif Z==3
            p(10) = prm.sepIntrusion;
            if R~=2 && Vn>0
                p(13) = prm.limbHazardSep;
            end
        end
    else
        p(5) = prm.physDown*prm.absentRecovery*(Fp>0);
        p(7) = prm.mentDown*prm.absentRecovery*(Fm>0);
    end
    % ---------------- predicted geometry: robot motion (travel, no jump)
    if a==3 && Z==3
        p(11) = prm.pApproach;
    elseif a==9 && (Z==1 || Z==2)
        p(11) = prm.pRetreat;
    end
    % ---------------- predicted geometry: environmental change (independent of the action)
    if hazard
        p(12) = prm.pWithdraw;
    elseif M==2 && Z==1 && H==1
        p(12) = prm.pLeaveEnvelope;
    elseif M==2 && Z==2 && H==1
        p(12) = prm.pReturnEnvelope;
    elseif R==1 && Z==3
        p(12) = prm.pRiskDecay;
    end
    if sum(p) > 1
        p = p/sum(p);
    end
    p(1) = max(1 - sum(p(2:nO)), 0);    % no negative round-off
    p = p/sum(p);
    t(nO*(a-1)+1:nO*a) = p;
end
end
