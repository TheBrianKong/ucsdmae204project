function T= FwdKinBody(M,B,theta)
%{
Fwd kinematics in body frame
T: end effector config in SE(3)
M: end effector zero-position config (T_sb when theta=0s)
B: 6xn list of body twists, each column is a twist [omega; v]
theta: list of thetas
%}
% simple anon function
skew = @(x) [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];
T=M;
for i=1:length(theta) % M*exp(s1 th1)*...*exp(s_n th_n)
    omega = B(1:3,i); v = B(4:6,i);
    S_mat = [skew(omega),v; zeros(1,4)];
    T= T*expm(S_mat*theta(i));
end
end