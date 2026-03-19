function T = T_se3(R,p)
%T_SE3 get 4x4 transformation matrix from R (3x3) and p(3x1)
% yes this used to be another anon function
T = [R, p; zeros(1,3) 1];
end

