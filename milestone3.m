%% TEST INPUTS FROM PROMPT
addpath("Functions\")
% (phi, x, y, th1, th2, th3, th4, th5) = (0, 0, 0, 0, 0, 0.2, -1.6, 0)
test_state = [0; 0; 0; 0; 0; 0.2; -1.6; 0; 0; 0; 0; 0]; 

% Reference frames (Xd and Xd_next)
Xd = [0 0 1 0.5; 0 1 0 0; -1 0 0 0.5; 0 0 0 1];
Xd_next = [0 0 1 0.6; 0 1 0 0; -1 0 0 0.3; 0 0 0 1];

% true frame X (calc'ed from fwd kinematics)
X = [0.170 0 0.985 0.387; 0 1 0 0; -0.985 0 0.170 0.570; 0 0 0 1];

% control params
Kp = zeros(6); % Test with zero gains first
Ki = zeros(6);
dt = 0.01;
int_err = zeros(6,1);

% run function
[Ve, controls] = FeedbackControl(X, Xd, Xd_next, Kp, Ki, dt, int_err, test_state);

% compare results
fprintf('Expected Vd: [0, 0, 0, 20, 0, 10]\n');
fprintf('Your Ve:     [%s]\n', num2str(Ve', '%.3f '));

fprintf('\nExpected (u, theta_dot):\n');
fprintf('157.2, 157.2, 157.2, 157.2, 0, -652.9, 1398.6, -745.7, 0\n');
fprintf('Our controls:\n');
fprintf('%s\n', num2str(controls', '%.1f '));