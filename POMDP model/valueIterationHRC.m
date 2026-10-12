clc;close all;
% value iteration of the underlying MDP (synthetic parameters); the online
% planner uses these values through the QMDP approximation (pomdpActionHRC)
[R,MK] = rewardHRC();     % cost of every state-action and allowed actions [34992 x 10]
N = NextStateHRC();       % successor states [34992 x 130]
T = TransProbHRC();       % successor probabilities [34992 x 130]
prm = stateDefinitionHRC();
gamma = prm.gamma;
V = zeros(length(R),1);
P = V;
itr = 0;
err = 1e2;
while (abs(err)>1e-6)
    Vp = V;
    for i = 1:length(R)
        v = V(N(i,:))';
        V(i) = vcalcHRC(gamma,v,T(i,:),R(i,:),MK(i,:));
    end
    itr = itr+1
    err = norm(V-Vp)
end
for j = 1:length(R)
    v = V(N(j,:))';
    P(j) = pcalcHRC(gamma,v,T(j,:),R(j,:),MK(j,:));
end
save('PHRC','P');
save('VHRC','V');
