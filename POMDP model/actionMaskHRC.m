function [mask, modeMask] = actionMaskHRC(M, Z)
% mask(a) = 1 if action a is allowed in task mode M with the HAND in zone Z
% modeMask(a) = 1 if action a belongs to the action set of mode M
%   solo          [1 2 8 9 10]
%   assistive     [3 4 6 8 9 10]
%   collaborative [5 6 7 8 10]  (no approach or retreat)
% Zone rules (actionDefinitionHRC, act.allowed):
%   Z = 0          -> only the protective stop
%   collaborative  -> 5, 6 and 8 only with the palm in the envelope (Z = 1);
%                     otherwise only the stopped wait 7 (or 10)
%   solo           -> normal speed (1) and reallocation (8) only with the hand separated
% Z = NaN (unknown hand zone) -> only the protective stop
act = actionDefinitionHRC();
modeMask = zeros(1,10);
if M==0
    modeMask(act.solo) = 1;
elseif M==1
    modeMask(act.assist) = 1;
else
    modeMask(act.collab) = 1;
end
if isnan(Z)
    mask = zeros(1,10);
    mask(10) = 1;
    return
end
mask = modeMask.*act.allowed{M+1}(:,Z+1)';
