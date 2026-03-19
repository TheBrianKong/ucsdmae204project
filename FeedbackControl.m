function [Ve,ctrls] = FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt, int_err,state)
%FEEDBACKCONTROL Feed fwd + PI feedback control to calculate required twist of end effector
% Inputs:
%   X: current config of end effector: Tse, 4x4
%   Xd: current reference config of ee Tse_d
%   Xd_next: next reference config
%   K_p, Ki: 6x6 PI gain matrices
%   dt: time step
% Outputs:
%   Ve: commanded ee twist in {e} frame (6x1)
%   controls: 9x1 vector velocity ctrl on wheels and joints

%{
EXAMPLE (milestone3.m)
test_state = [0; 0; 0; 0; 0; 0.2; -1.6; 0; 0; 0; 0; 0]; 
% Reference frames (Xd and Xd_next)
Xd = [0 0 1 0.5; 0 1 0 0; -1 0 0 0.5; 0 0 0 1];
Xd_next = [0 0 1 0.6; 0 1 0 0; -1 0 0 0.3; 0 0 0 1];
% true frame X
X = [0.170 0 0.985 0.387; 0 1 0 0; -0.985 0 0.170 0.570; 0 0 0 1];
Kp = zeros(6);
Ki = zeros(6);
dt = 0.01;
int_err = zeros(6,1);
[Ve, controls] = FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt, int_err, test_state);
%}


% feed fwd in {d}
% 1st order finite diff of [Vd]= Xd^-1(t) * \dot {X_d(t)}
global Je_current;
Vd_mat = (1/dt) * MatrixLog6(Xd\ Xd_next);
Vd = [Vd_mat(3,2); Vd_mat(1,3); Vd_mat(2,1); ... % Angular
      Vd_mat(1,4); Vd_mat(2,4); Vd_mat(3,4)];   % Linear
% error in {e} frame
% [Xerr] = log(X^-1 * Xd)
Xerr_mat = MatrixLog6(X\Xd);
Xerr = [Xerr_mat(3,2); Xerr_mat(1,3); Xerr_mat(2,1); 
        Xerr_mat(1,4); Xerr_mat(2,4); Xerr_mat(3,4)];

% Vd -> Ve using adjoint
Ad_Xinv_Xd = Ad_T(X\Xd);

% Compute Commanded Twist Ve
Ve = Ad_Xinv_Xd * Vd + Kp * Xerr + Ki * (int_err + Xerr * dt);

% system home configuration
l = 0.47/2; w = 0.3/2; r = 0.0475;
Tb0 = [1, 0, 0, 0.1662; 0, 1, 0, 0; 0, 0, 1, 0.0026; 0, 0, 0, 1];
M0e = [1, 0, 0, 0.0330; 0, 1, 0, 0; 0, 0, 1, 0.6546; 0, 0, 0, 1];
    
% arm screw axes Bi = [wi; vi] in {e}
Blist = [[0; 0; 1; 0; 0.0330; 0], ...
         [0; -1; 0; -0.5076; 0; 0], ...
         [0; -1; 0; -0.3526; 0; 0], ...
         [0; -1; 0; -0.2176; 0; 0], ...
         [0; 0; 1; 0; 0; 0]];

theta=state(4:8);
J_arm = J_body(Blist,theta);
% Base Jacobian J_base
F = r/4 * [-1/(l+w), 1/(l+w), 1/(l+w), -1/(l+w); 
            1, 1, 1, 1; 
           -1, 1, -1, 1];
F6 = [zeros(2,4); F(1,:); F(2:3,:); zeros(1,4)];
% T_0e depends on theta. fwd k
T0e = FwdKinBody(M0e, Blist, theta);
Tbe=Tb0 * T0e;
Teb = inv(Tbe);
% [Rbe, p_be] = TtoRp(Tb0 * T0e); % Tb;
% Teb = T_se3(Rbe.',-Rbe.'*p_be);
J_base = Ad_T(Teb) * F6;
Je = [J_base, J_arm];
Je_current = Je;
ctrls= pinv(Je, 1e-4) * Ve; 

% Reorder from [wheels; arms] to [arms; wheels]
ctrls = [ctrls(5:9); ctrls(1:4)]; 
end

