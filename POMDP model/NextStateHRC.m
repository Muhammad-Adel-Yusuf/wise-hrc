function N = NextStateHRC(S)
% N = NextStateHRC()    table of successor state indices of all states (34992 x 130)
% N = NextStateHRC(S)   successor row of one state S
%
% PLANNER MODEL (predicted geometry, not the measured geometry).
% Column 13*(a-1)+j is outcome j of action a (probabilities in TransProbHRC):
%   j = 1 nominal                 j = 8  attention worse
%   j = 2 instruction +1          j = 9  attention better
%   j = 3 instruction -1          j = 10 hand crosses the prohibited boundary (Z'=0, R'=2)
%   j = 4 physical fatigue +1     j = 11 robot motion changes the hand band
%   j = 5 physical fatigue -1     j = 12 environmental change by the human
%   j = 6 mental fatigue +1       j = 13 elbow/forearm/upper-arm hazard (Z' = Z, R' = 2)
%   j = 7 mental fatigue -1
% Every outcome sets the commanded motion V' = act.motion(a). Outcomes 1-9
% keep the hand zone Z. An unresolved hazard (Z = 0 or R = 2) stays in every
% outcome except the environmental clearance j = 12: the protective stop
% stops the robot but does not move the human away.
% The task mode M never changes here (it is set by the task layer).
prm = stateDefinitionHRC();
act = actionDefinitionHRC();
if nargin > 0
    N = nextRow(S, prm, act);
    return
end
N = zeros(prm.nS,10*prm.nOut);
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
                                    N(k,:) = nextRow([M I H Fp Fm A Z V R], prm, act);
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

function n = nextRow(S, prm, act)
M = S(1); I = S(2); H = S(3); Fp = S(4); Fm = S(5); A = S(6); Z = S(7); R = S(9);
nO = prm.nOut;
n = zeros(1,10*nO);
Inext = I+1; if I==6, Inext = 1; end   % last instruction -> next product
Iback = max(I-1,1);
% risk kept by the nominal outcome: an active risk persists, otherwise the
% risk cannot drop below the level implied by the hand band
if R==2
    Rn = 2;
else
    Rn = max(R, baseRisk(Z,H));
end
% environmental change (outcome 12)
if Z==0 || R==2
    if Z==0, Ze = prm.homeZone(M+1); else, Ze = Z; end   % human withdraws (not caused by the robot)
    Re = baseRisk(Ze,H);
elseif M==2 && Z==1
    Ze = 2; Re = baseRisk(2,H);                          % hand leaves the envelope
elseif M==2 && Z==2
    Ze = 1; Re = baseRisk(1,H);                          % hand returns to the envelope
elseif R==1 && Z==3
    Ze = 3; Re = 0;                                      % residual potential risk decays
else
    Ze = Z; Re = Rn;
end
for a = 1:10
    Vn = act.motion(a);
    c = nO*(a-1);
    n(c+1)  = S2indexHRC([M I H Fp Fm A Z Vn Rn]);                         % nominal
    n(c+2)  = S2indexHRC([M Inext H Fp Fm A Z Vn Rn]);                     % I+1
    n(c+3)  = S2indexHRC([M Iback H Fp Fm A Z Vn Rn]);                     % I-1
    n(c+4)  = S2indexHRC([M I H min(Fp+1,2) Fm A Z Vn Rn]);                % Fp+1
    n(c+5)  = S2indexHRC([M I H max(Fp-1,0) Fm A Z Vn Rn]);                % Fp-1
    n(c+6)  = S2indexHRC([M I H Fp min(Fm+1,2) A Z Vn Rn]);                % Fm+1
    n(c+7)  = S2indexHRC([M I H Fp max(Fm-1,0) A Z Vn Rn]);                % Fm-1
    n(c+8)  = S2indexHRC([M I H Fp Fm min(A+1,2) Z Vn Rn]);                % attention worse
    n(c+9)  = S2indexHRC([M I H Fp Fm max(A-1,0) Z Vn Rn]);                % attention better
    n(c+10) = S2indexHRC([M I H Fp Fm A 0 Vn 2]);                          % hand intrusion
    % robot motion changes the hand band (approach / retreat only)
    if act.target(a) >= 0 && Z~=0
        Zt = act.target(a);
        if R==2, Rt = 2; else, Rt = baseRisk(Zt,H); end
    else
        Zt = Z; Rt = Rn;
    end
    n(c+11) = S2indexHRC([M I H Fp Fm A Zt Vn Rt]);
    n(c+12) = S2indexHRC([M I H Fp Fm A Ze Vn Re]);                        % environmental change
    n(c+13) = S2indexHRC([M I H Fp Fm A Z Vn 2]);                          % limb hazard
end
end

function r = baseRisk(Z, H)
% risk implied by the hand band alone (planner approximation only; the
% observed risk comes from riskEstimateHRC with the full limb geometry)
if H==0
    r = 0;
elseif Z==0
    r = 2;
elseif Z==3
    r = 0;
else
    r = 1;
end
end
