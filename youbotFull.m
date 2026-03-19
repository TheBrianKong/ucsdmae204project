%% .1 Initialization
dt = 0.01;
v_max = 50; 
k = 3; % One config per dt
addpath("Functions\") % change based on where the Functions folder is
test_case = 'overshoot'; % Options: 'best', 'overshoot', 'new task','open'

% start and end cube configurations (Tsc)
Tsc_initial = [1, 0, 0, 1; 0, 1, 0, 0; 0, 0, 1, 0.025; 0, 0, 0, 1];
Tsc_final   = [0, 1, 0, 0; -1, 0, 0, -1; 0, 0, 1, 0.025; 0, 0, 0, 1];

% true init state (q, theta, wheels)
current_state = [0.2; 0.4; -pi/4; 0; 0; 0; 0; 0; 0; 0; 0; 0];


% initial reference of {e} (Tse_ref)
Tse_ref_initial = [0, 0, 1, 0; 0, 1, 0, 0; -1, 0, 0, 0.5; 0, 0, 0, 1];

% gripper frame relative to cube (grasp and standoff)
R_ce =  @(th) [cos(th) 0 sin(th); 0 1 0; -sin(th) 0 cos(th)]; 
% orient to face down; rotate by 3/4 pi about y_e
Tce_grasp = [R_ce(3/4*pi), [0;0;0]; 0 0 0 1];
Tce_standoff= [R_ce(3/4*pi), [0;0;0.05]; 0 0 0 1];


switch test_case
    case 'best'
        Kp = 3 * eye(6); 
        Ki = 0.015 * eye(6);
        % Standard cube locations
    case 'overshoot'
        Kp = 10 * eye(6); % High P-gain causes aggressive correction
        Ki = 5 * eye(6);  % High I-gain causes the oscillation/overshoot
    case 'new task'
        Kp = 4 * eye(6); 
        Ki = 0.01 * eye(6);
        % NEW cube target and goals
        Tsc_initial = [1, 0, 0, 0.5; 0, 1, 0, 2; 0, 0, 1, 0.025; 0, 0, 0, 1];
        Tsc_final   = [0, 1, 0, 0; -1, 0, 0, -1; 0, 0, 1, 0.025; 0, 0, 0, 1];
    case 'open'
        % open loop
        Kp = zeros(6);
        Ki = zeros(6);
end
%% 2. Generate Trajectory
% Nx13 matrix [R11..R33, Tx, Ty, Tz, gripper state]
traj = TrajectoryGenerator(Tse_ref_initial, Tsc_initial, Tsc_final, Tce_grasp, Tce_standoff, k);
N = size(traj, 1);
global Je_current;
mu_log = zeros(N-1,2); % store both angular and linear
state_log = zeros(N, 13);
error_log = zeros(N-1, 6);
int_err = zeros(6, 1);

Tsb= @(q) [cos(q(1)) -sin(q(1)) 0 q(2); sin(q(1)) cos(q(1)) 0 q(3); 0 0 1 0.0963; 0 0 0 1];
Tb0 = [1, 0, 0, 0.1662; 0, 1, 0, 0; 0, 0, 1, 0.0026; 0, 0, 0, 1];
M0e = [1, 0, 0, 0.0330; 0, 1, 0, 0; 0, 0, 1, 0.6546; 0, 0, 0, 1];
% arm screw axes Bi = [wi; vi] in {e}
Blist = [[0;0;1;0;0.033;0],[0;-1;0;-0.5076;0;0], [0;-1;0;-0.3526;0;0], [0;-1;0;-0.2176;0;0], [0;0;1;0;0;0]];
%% Simulation loop
for i = 1:N-1
    % A. Current EE Pose X (Forward Kinematics)
    phi = current_state(1); x = current_state(2); y = current_state(3);
    theta = current_state(4:8);
    % Tse(theta)
    X = Tsb(current_state(1:3)) * Tb0 * FwdKinBody(M0e, Blist, theta);
    
    % B. References from Trajectory
    % Use ' (transpose) if your TrajectoryGenerator outputs rows R11, R12, R13...
    Xd = [reshape(traj(i, 1:9), 3, 3)', traj(i, 10:12)'; 0 0 0 1];
    Xd_next = [reshape(traj(i+1, 1:9), 3, 3)', traj(i+1, 10:12)'; 0 0 0 1];
    % Control & manipulability
    [Ve, controls] = FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt, int_err, current_state);
    
    mu_log(i,1) = sqrt(det(Je_current(1:3,:)*Je_current(1:3,:)')); % angular
    mu_log(i,2) = sqrt(det(Je_current(4:6,:)*Je_current(4:6,:)')); % linear

    % error and integral update
    Xerr_mat = logm(X \ Xd);
    Xerr = [Xerr_mat(3,2); Xerr_mat(1,3); Xerr_mat(2,1); Xerr_mat(1,4); Xerr_mat(2,4); Xerr_mat(3,4)];
    int_err = int_err + Xerr * dt;
    
    % next state
    current_state = NextState(current_state, controls, dt, v_max);
    
    % log states and error
    state_log(i, :) = [current_state', traj(i, 13)];
    error_log(i, :) = Xerr';
end

state_log(N, :) = [current_state', traj(N, 13)]; % Final gripper state
writematrix(state_log, 'youBot_simulation.csv');

%% Results plots
% Define a wider figure size: [left, bottom, width, height]
wide_pos = [100, 100, 1000, 400]; 

% --- Figure 1: Error Plot (Tiled) ---
figure(1);
set(gcf, 'Position', wide_pos);
t = tiledlayout(2,1);

% Top Tile: Linear Velocities (v)
nexttile;
plot(0:dt:(N-2)*dt, error_log(:, 4:6), 'LineWidth', 1);
ylabel('Linear Error');
legend('v_x','v_y','v_z', 'Location', 'eastoutside');
grid on;
%xlim([0 2]);
% Bottom Tile: Angular Velocities (omega)
nexttile;
plot(0:dt:(N-2)*dt, error_log(:, 1:3), 'LineWidth', 1);
ylabel('Angular Error');
xlabel('Time (s)');
legend('\omega_x','\omega_y','\omega_z', 'Location', 'eastoutside');
grid on;

title(t, ['Xerr Convergence - ', test_case]);
%xlim([0 2]);
saveas(gcf, ['error_', test_case, '.png']);

% --- Figure 2: Manipulability Plot (Wide & Semilog) ---
figure(2);
set(gcf, 'Position', wide_pos);
hold on;

yyaxis left;
semilogy(0:dt:(N-2)*dt, mu_log(:, 1), 'LineWidth', 1.5);
ylabel('Angular Manipulability \mu_{\omega}');
set(gca, 'YColor', [0 0.4470 0.7410]); % Match line color

yyaxis right;
semilogy(0:dt:(N-2)*dt, mu_log(:, 2), 'LineWidth', 1.5);
ylabel('Linear Manipulability \mu_{v}');
set(gca, 'YColor', [0.8500 0.3250 0.0980]); % Match line color

xlabel('Time (s)');
title(['Manipulability (Log Scale) - ', test_case]);
grid on;
saveas(gcf, ['manip_', test_case, '.png']);