function V = vcalcHRC(gamma,v,T,R,mk)
% v  : values of the 130 successor states (13 outcomes x 10 actions), T : their probabilities
% R  : cost of the 10 actions in this state, mk : allowed actions
% actions: sc ss ap ah cn cs ch ra rt ps (see actionDefinitionHRC)
nO = length(T)/10;              % outcomes per action (13)
u = zeros(1,10);
for a = 1:10
    c = nO*(a-1)+1:nO*a;
    if mk(a)==1
        u(a) = -R(a) + gamma*(T(c)*v(c)');
    else
        u(a) = -100000;
    end
end
V = max(u);
