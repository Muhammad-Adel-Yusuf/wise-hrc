function d = segmentDistanceHRC(p1,q1,p2,q2)
% shortest distance between segment p1-q1 and segment p2-q2 (3-D, meters)
% (closest-point method for two segments, handles degenerate segments)
d1 = q1(:) - p1(:);
d2 = q2(:) - p2(:);
r  = p1(:) - p2(:);
a = d1'*d1; e = d2'*d2; f = d2'*r;
tol = 1e-12;
if a <= tol && e <= tol
    d = norm(p1(:) - p2(:));
    return
end
if a <= tol
    s = 0;
    t = min(max(f/e,0),1);
else
    c = d1'*r;
    if e <= tol
        t = 0;
        s = min(max(-c/a,0),1);
    else
        b = d1'*d2;
        den = a*e - b*b;
        if den > tol
            s = min(max((b*f - c*e)/den,0),1);
        else
            s = 0;                       % parallel segments
        end
        t = (b*s + f)/e;
        if t < 0
            t = 0;
            s = min(max(-c/a,0),1);
        elseif t > 1
            t = 1;
            s = min(max((b - c)/a,0),1);
        end
    end
end
d = norm((p1(:) + d1*s) - (p2(:) + d2*t));
