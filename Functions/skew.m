function X = skew(x)
%SKEW 3x3 skew symmetric matrix from vector R^3
% originally was an anon funct but got annoying.
% skew = @(x) [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];
X = [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];
end

