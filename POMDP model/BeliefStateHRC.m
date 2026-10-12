function [b,il,bp] = BeliefStateHRC(b,a,Sp,S,O,ev,tot)
% Belief over the hidden human state x = [Fp Fm A] (27 values, j = 9*Fp + 3*Fm + A + 1)
%   prediction : bp(x') = sum_x T(x'|x,a,c) b(x)        (TransProbHRC / NextStateHRC, context Sp)
%   update     : b(x')  = eta * O(o|x') * bp(x')         (observationLikelihoodHRC)
% b   : previous belief (empty at the first epoch -> prior)
% a   : action applied at the previous epoch (after the safety supervisor);
%       a = 0 (task-layer transfer, no POMDP action) is predicted with the
%       neutral slow-motion action 2, which has no direct effect on the human
% Sp  : previous observed state (context of the prediction)
% S   : current observed context (only M I H Z V R are used; Z and R are the
%       OBSERVED hand zone and risk, the belief itself only covers Fp Fm A)
% O   : current observation, ev : events of the previous epoch, tot : time on task
% il  : state indices of the 27 hidden combinations in the current context
prm = stateDefinitionHRC();
if isempty(b)
    bp = prm.prior;
else
    if a==0, a = 2; end
    bp = zeros(27,1);
    nO = prm.nOut;
    c = nO*(a-1)+1:nO*a;
    for fp = 0:2
        for fm = 0:2
            for x = 0:2
                j = 9*fp + 3*fm + x + 1;
                if b(j)==0, continue; end
                Sx = [Sp(1:3) fp fm x Sp(7:9)];
                t = TransProbHRC(Sx,ev,tot);
                n = NextStateHRC(Sx);
                for q = c
                    Sn = index2SHRC(n(q));
                    jn = 9*Sn(4) + 3*Sn(5) + Sn(6) + 1;
                    bp(jn) = bp(jn) + b(j)*t(q);
                end
            end
        end
    end
end
bp = bp/sum(bp);
L = observationLikelihoodHRC(O);
b = L.*bp;
if sum(b) > 0
    b = b/sum(b);
else
    b = bp;                 % observation impossible under the model: keep the prediction
end
il = zeros(27,1);
for fp = 0:2
    for fm = 0:2
        for x = 0:2
            il(9*fp + 3*fm + x + 1) = S2indexHRC([S(1:3) fp fm x S(7:9)]);
        end
    end
end
