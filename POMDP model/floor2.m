function k = floor2(i,j)
k = floor(i/j);
if rem(i,j)==0
    k = k - 1;
end