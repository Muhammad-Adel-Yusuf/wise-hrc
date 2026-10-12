function [a,score] = pomdpActionHRC(b,il,P,V,N,T,R,MK,taskMask)
% nominal action from the belief with the QMDP approximation of the POMDP
% (the MDP Q-values are averaged over the belief; QMDP assumes the hidden
% state becomes known after one step, so it does not plan information-
% gathering actions):
%   score(a) = sum_j b(j) * ( z*(P(il(j))==a) + Q(il(j),a) )
%   Q(i,a)   = -R(i,a) + gamma * sum T(i,c)*V(N(i,c)),  c = outcomes of action a
% z = prm.z = 0 gives pure QMDP; z > 0 adds the vote of the MDP policy P
% (as in trajPOMDP_HRC1). Actions not allowed in the OBSERVED context (mode and
% hand zone, MK) get -Inf. The result is only a NOMINAL action: the safety
% supervisor decides independently.
% taskMask (optional, 1x10): actions the task layer allows at this moment.
prm = stateDefinitionHRC();
gamma = prm.gamma;
nO = prm.nOut;
z = prm.z;
score = zeros(10,1);
for j = 1:27
    if b(j)==0, continue; end
    i = il(j);
    for u = 1:10
        c = nO*(u-1)+1:nO*u;
        q = -R(i,u) + gamma*(T(i,c)*V(N(i,c)));
        score(u) = score(u) + (z*(P(i)==u) + q)*b(j);
    end
end
mk = MK(il(1),:);              % same context for all 27 hidden states
if nargin > 8 && ~isempty(taskMask)
    mk = mk.*taskMask;         % task-layer restriction (e.g. handover hold: only 4 and 10)
    if ~any(mk), mk(10) = 1; end
end
score(mk==0) = -Inf;
a = findmax(score);
