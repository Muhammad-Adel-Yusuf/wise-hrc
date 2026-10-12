function i = S2indexHRC(S)
% S = [M I H Fp Fm A Z V R], I = 1..6, all other variables start at 0
% sizes [3 6 2 3 3 3 4 3 3], the last variable changes fastest
i = S(9)+1 + 3*S(8) + 3^2*S(7) + 3^2*4*S(6) + 3^3*4*S(5) + 3^4*4*S(4) + 3^5*4*S(3) ...
    + 3^5*4*2*(S(2)-1) + 3^5*4*2*6*S(1);
