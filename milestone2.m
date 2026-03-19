% test of state iteration
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

% VIDEO LINK
% https://youtu.be/j2PR-Uw2rl8