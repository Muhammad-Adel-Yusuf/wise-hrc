function m = findmax(v)
n = max(v);
for i = 1:length(v)
    if v(i)==n
        m = i;
    end
end