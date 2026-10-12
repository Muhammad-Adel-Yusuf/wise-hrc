function [R, MK] = rewardHRC()

prm = stateDefinitionHRC();
act = actionDefinitionHRC();
R = zeros(prm.nS,10);
MK = zeros(prm.nS,10);
k = 1;
for M = 0:2
    for I = 1:6
        for H = 0:1
            for Fp = 0:2
                for Fm = 0:2
                    for A = 0:2
                        for Z = 0:3
                            for V = 0:2
                                for Rk = 0:2
                                    MK(k,:) = actionMaskHRC(M,Z);
                                    for a = 1:10
                                        R(k,a) = cost([M I H Fp Fm A Z V Rk], a, prm, act);
                                    end
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

function C = cost(S, a, prm, act)
M = S(1); I = S(2); H = S(3); Fp = S(4); Fm = S(5); A = S(6); Z = S(7); V = S(8); R = S(9);
Vn = act.motion(a);
inten = prm.motionIntensity(Vn+1);
zb = act.target(a); if zb < 0, zb = Z; end      % hand band the action aims at
lambda = prm.lambda(M+1);
fatigue = prm.wPhysical*Fp/2 + prm.wMental*Fm/2;
near = (H==1) && (Z~=3);                          % hand not separated from the robot
need = prm.assistNeeded(I);
% task progress loss (unnecessary delay)
c1 = prm.taskLoss(M+1,a);
if a==8 && M==1 && need && Z==2
    c1 = 0;                                       % assistance delivered in the assistive zone
end
if M==2 && H==0 && any(a==[5 6 8])
    c1 = c1 + prm.noHumanCost;                    % collaborative work without the human
end
% time
c2 = prm.timeCost + prm.holdCost*(Vn==0);
% fatigue
c3 = lambda*fatigue;
% hand zone: solo -> separated, assistive -> assistive region while help is needed,
% collaborative -> collaborative envelope
pref = prm.homeZone(M+1);
if M==1 && ~need, pref = 3; end
c4 = prm.zoneCost*(zb~=pref);
% motion: speed near a fatigued human + smoothness
c5 = lambda*fatigue*inten*near + prm.smoothCost*abs(Vn-V);
% interaction: potential risk and attention while moving near the human
c6 = (R==1)*prm.riskMotion(Vn+1) + near*prm.attMotion(A+1)*inten;
% safety: moving in a hazard, or stopping without a reason
hazard = (R==2) || (Z==0);
c7 = hazard*(a~=10)*prm.safetyCost + (~hazard)*(a==10)*prm.stopCost;
% solo: motion near the human by fatigue level (none for low fatigue)
c8 = (M==0)*near*prm.soloFatigueMotion(max(Fp,Fm)+1)*inten;
C = c1 + c2 + c3 + c4 + c5 + c6 + c7 + c8;
end
