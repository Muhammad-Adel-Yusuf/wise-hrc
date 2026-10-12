function L = observationLikelihoodHRC(O)
% L(j) = P(O | Fp, Fm, A) for the 27 hidden states, j = 9*Fp + 3*Fm + A + 1
% O(1) physical fatigue, O(2) mental fatigue, O(3) attention, O(4) presence,
% O(7) HUMAN-STATE confidence (fatigue/attention estimation; NOT the safety /
% geometry confidence O(12), which never enters the belief), O(8) face detected
% - a missing (NaN) channel gives no information
% - face not detected -> attention gives no information (it is NOT distraction)
% - low human-state confidence flattens the confusion matrices: C = conf*C + (1-conf)/3
% - human absent -> no information
prm = stateDefinitionHRC();
L = ones(27,1);
if O(4)==0
    return
end
conf = O(7);
if isnan(conf), conf = 1; end
conf = min(max(conf,0.05),1);
Cp = conf*prm.confPhys + (1-conf)/3;
Cm = conf*prm.confMent + (1-conf)/3;
Ca = conf*prm.confAtt  + (1-conf)/3;
oA = O(3);
if O(8)==0, oA = NaN; end
for fp = 0:2
    for fm = 0:2
        for a = 0:2
            j = 9*fp + 3*fm + a + 1;
            if ~isnan(O(1)), L(j) = L(j)*Cp(fp+1,O(1)+1); end
            if ~isnan(O(2)), L(j) = L(j)*Cm(fm+1,O(2)+1); end
            if ~isnan(oA),   L(j) = L(j)*Ca(a+1,oA+1);    end
        end
    end
end
