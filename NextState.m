function state_new = NextState(state,v, dt, v_max)
%NEXTSTATE 1st order Euler step for youBot
% Inputs:
%   state: current 12x1 state: [phi, x, y, th1~5, w1~4]
%   v: 9x1 arm joint & wheel angular velocities
%   dt: time step size (keep reasonable, it's using euler..)
%   v_max: speed limit for abs velocity
% Output: 
%   state_new: 12x1 state for updated time (t+dt)

%{
EXAMPLE (milestone2.m)
dt = 0.01;
tf = 10;
N = tf/dt;
v_max = 20; % rad/s
state = zeros(12,1);
v_test=  [zeros(1,5) -2.5 2.5 -2.5 2.5].';
datalog = zeros(N,13); % because we need to add gripper
for i = 1:N
    state = NextState(state,v_test,dt,v_max);
    datalog(i,:)= [state.',0];
end
writematrix(datalog,'milestone2_ctrltest.csv');
%}

v(v>v_max) = v_max;
v(v<-v_max) = -v_max;

%make it easier to read by extracting velocity components
q = state(1:3); % phi x y
arm_th = state(4:8); % 5 joints
wh_angle = state(9:12); % considered replacing 12 → end, but
% that level of redundancy is overkill, i expect the rest to break
arm_speed = v(1:5);
wh_speed = v(6:9);

arm_th_new = arm_th + arm_speed*dt;
wh_angle_new = wh_angle + wh_speed*dt;

% distances of wheel to chassis frame {b}, and wheel radius (m)
l = .47/2; w = 0.3/2; r = .0475;

% chassis velocity represented as planar twist [om_bz, v_bx, v_by] wrt {b}
% Vb = inv(H)*u, u = wheel speeds, H^-1(0) b/c omnidirectional
%H = 1/r*[-l-w, 1, -1; l+w, 1 1; l+w, 1, -1; -l-w 1 1]; Vb= H\wh_speed;
F = r/4*[-1/(l+w) 1/(l+w) 1/(l+w) -1/(l+w); 1 1 1 1; -1 1 -1 1]; % eqn 13.33, MR
Vb = F*wh_speed;

% if i choose to do euler step for twist and just have each step as lines
dq_b = Vb*dt;
if abs(dq_b(1))>1e-6 % mR eqn 13.35
    dq_b = [dq_b(1);
            (dq_b(2)*sin(dq_b(1)) + dq_b(3)*(cos(dq_b(1)) - 1)) / dq_b(1);
            (dq_b(3)*sin(dq_b(1)) + dq_b(2)*(1 - cos(dq_b(1)))) / dq_b(1)];
end
% transform to {s}
dq = [1 0 0; 0 cos(q(1)) -sin(q(1)); 0 sin(q(1)) cos(q(1)) ]*dq_b; 
q_new = q + dq;

state_new = [q_new; arm_th_new; wh_angle_new];
end

