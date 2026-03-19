function traj = CartesianTrajectory(Xstart, Xend, Tf, N)
% take start and end end effector frame configs, and number of points (N) in trajectory

% total time frame (Tf) is implicit by N but this 
% quintic time scaling
[Rstart, pstart] = TtoRp(Xstart);
[Rend, pend] = TtoRp(Xend);
timegap = Tf / (N - 1);
% rotation displacement
R_disp = logm(Rstart' * Rend);
P_disp = pend - pstart;

traj = cell(1, N);

for i = 1:N
    % current time fraction (0 to 1)
    % t = timegap * (i-1);
    % tau = t/Tf, 
    tau = (i - 1) / (N - 1);
    % Quintic time scaling s(tau) for no jerking
    s = 10*tau^3 - 15*tau^4 + 6*tau^5;
    % interpolate R and p
    Ri = Rstart * expm(R_disp * s);
    pi = pstart + s * P_disp;
    traj{i} = [Ri, pi; 0, 0, 0, 1];
end
end