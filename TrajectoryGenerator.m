function [Tse_t] = TrajectoryGenerator(Tse_0,Tsc_0,Tsc_n,Tce_grasp,Tce_standoff,k)
% Input Arguments:
% - Tse_0, Tsc_0 are the initial transformation matrices for end effector {e} and cube {c}
% - Tsc_N is the final transformation matrix for cube wrt s
% - Tce_grasp: config of {e} relative to {c} while grasping
% - Tce_standoff: config of {e} relative to {c} while in standby/ above 
% - k: number of trajectory reference configurations per 0.01 s 
% Output Arguments:
% - Tse_t: N=t*k/0.01 configurations of {e} relative to {s} at any instant in time

%{
EXAMPLE (milestone1.m)
% Tsc_0 of 5 cm cube on (1,0,0)
Tsc_0 = [1 0 0 1; 0 1 0 0; 0 0 1 0.025; 0 0 0 1];
% Tsc_n of 5 cm cube on (0,-1,-pi/2), which is rotated CW 90 deg.
Tsc_n = [0 1 0 0; -1 0 0 -1; 0 0 1 0.025; 0 0 0 1];

% youbot frames
q = [0; 0; 0]; % phi, x, y
Tsb= @(q) [cos(q(1)) -sin(q(1)) 0 q(2); sin(q(1)) cos(q(1)) 0 q(3); 0 0 1 0.0963; 0 0 0 1];
Tb0 = [1 0 0 0.1662; 0 1 0 0; 0 0 1 0.002; 0 0 0 1];
T0e_0 = [1 0 0 0.033 ;0 1 0 0 ;0 0 1 0.6546 ;0 0 0 1]; 

Tse_0 = Tsb(q)*Tb0*T0e_0;
% cube-gripper translation
R_ce =  @(th) [cos(th) 0 sin(th); 0 1 0; -sin(th) 0 cos(th)];
Tce_grasp = [R_ce(3/4*pi), [0;0;0]; 0 0 0 1];
Tce_standoff= [R_ce(3/4*pi), [0;0;0.1]; 0 0 0 1];
k=1;
Tse_t = TrajectoryGenerator(Tse_0,Tsc_0,Tsc_n,Tce_grasp,Tce_standoff,k);
%}

% Tse  = Tsc * Tce
seg_duration =ones(8,1); % adjust to change duration of each segment
% objective of each segment
T_standoff_0 = Tsc_0 * Tce_standoff;
T_grasp_0 = Tsc_0 * Tce_grasp;
T_standoff_n = Tsc_n * Tce_standoff;
T_grasp_n = Tsc_n * Tce_grasp;
% ordered sequence
T_list_start = {Tse_0; T_standoff_0; T_grasp_0; T_grasp_0; T_standoff_0; T_standoff_n; T_grasp_n; T_grasp_n};
T_list_end = {T_standoff_0; T_grasp_0; T_grasp_0; T_standoff_0; T_standoff_n; T_grasp_n; T_grasp_n; T_standoff_n};
grab = [0;0;1;1;1;1;0;0];
Tse_t=[];
for i = 1:length(grab)
    N = seg_duration(i)*k/0.01;
    seg_trajectory = CartesianTrajectory(T_list_start{i}, T_list_end{i}, seg_duration(i), N);
    for j = 1:length(seg_trajectory)
        T = seg_trajectory{j};
        Tse_i=[T(1,1:3), T(2,1:3), T(3,1:3), T(1:3,4).', grab(i)];
        Tse_t= [Tse_t;Tse_i];
    end
end
writematrix(Tse_t, 'reference_trajectory.csv');
end

