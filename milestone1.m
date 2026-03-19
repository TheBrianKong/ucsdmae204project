addpath("Functions\");
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
% gripper frame relative to cube (grasp and standoff)
R_ce =  @(th) [cos(th) 0 sin(th); 0 1 0; -sin(th) 0 cos(th)]; % orient to face down; rotate by 3/4 pi about y_e
Tce_grasp = [R_ce(3/4*pi), [0;0;0]; 0 0 0 1];
Tce_standoff= [R_ce(3/4*pi), [0;0;0.1]; 0 0 0 1];
k=1;
Tse_t = TrajectoryGenerator(Tse_0,Tsc_0,Tsc_n,Tce_grasp,Tce_standoff,k);
%% YOUTUBE LINK FOR SUCCESSFUL SIMULATION
% https://youtu.be/OJ1wzuUw9Yk

