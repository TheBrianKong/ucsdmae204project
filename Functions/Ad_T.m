function Adjoint = Ad_T(varargin)
%AD_T 6x6 adjoint matrix from either:
% input: rotation matrix R and frame origin p
% input: transformation matrix T

% if I ever decide to divorce this file from the rest of functions
% Ad_T= @(R,p) [R zeros(3,3); skew(p)*R R];
% skew = @(x) [0, -x(3), x(2); x(3), 0, -x(1); -x(2), x(1) 0];
if nargin ==1
    T = varargin{1};
    R = T(1:3,1:3);
    p = T(1:3,4);
elseif nargin ==2
    R = varargin{1};
    p = varargin{2};
else
    error("Ad_T requires 1 or 2 input args: (T) or (R,p)");
end
% so apparently matlab likes being quirky and have a function-level scope, not block.
Adjoint = [R zeros(3,3); skew(p)*R R];
end

