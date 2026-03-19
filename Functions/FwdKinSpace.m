function T = FwdKinSpace(M,S,theta)
%{
Fwd kinematics in space frame
T: end effector config in SE(3)
M: end effector zero-position config (T_sb when theta=0s)
S: 6xn list of spatial twists, each column is a twist [omega; v]
theta: list of thetas
%}
skew = @(x) [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];
T=M;
for i=length(theta):-1:1 % exp(s_1 th_1)*...*exp(s_n th_n)*M
    omega = S(1:3,i); v = S(4:6,i);
    S_mat = [skew(omega),v; zeros(1,4)];
    T= expm(S_mat*theta(i))*T;
end
end
