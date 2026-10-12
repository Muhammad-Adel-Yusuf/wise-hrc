function P = pcalcHRC(gamma,v,T,R,mk)
% best action of one state (same terms as vcalcHRC)
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
P = findmax(u);
