function [thetalist,success] = IKinSpace(Slist,M,T,thetalist0,e_om,e_v)
% IKINSPACE inverse kinematics from home config (M) to target config (T),
% converge w/ Newton-Raphson with (assumed close) init guess thetalist0
thetalist = thetalist0;
maxiterations = 20;
% history=[];
for i = 0:maxiterations
    % current config
    Tsb = FwdKinSpace(M,Slist,thetalist);
    % error twist
    T_err = [Tsb(1:3,1:3).', -Tsb(1:3,1:3).'*Tsb(1:3,4); 0 0 0 1] * T;
    V_mat = MatrixLog6(T_err); % V_b
    % convert se(3) to vector [omega; v] and bring it to V_s
    Vs = Ad_T(Tsb)*[V_mat(3,2); V_mat(1,3); V_mat(2,1); V_mat(1:3, 4)];
    % save to history
    % history = [history; thetalist.'];
    % report
    % values to check convergence
    omega_mag = norm(Vs(1:3)); v_mag = norm(Vs(4:6));

    % fprintf('Iteration %d:\n', i);
    % fprintf('\ttheta: %s\n', mat2str(thetalist.'));
    % fprintf('\tT_sb:\n'); disp(Tsb);
    % fprintf('\tV_b: %s\n', mat2str(Vs', 4));
    % fprintf('\t||omega_b||: %1.6f, ||vb||: %1.6f\n', omega_mag, v_mag);
    % fprintf('------\n');
    % 
    % exit cond for success
    if (omega_mag < e_om && v_mag < e_v)
        fprintf('Convergence achieved at iteration %d\n', i);
        success = true;
        % csvwrite('joint_history.csv', history);
        return;
    end
    
    % update via NR method
    Js = J_space(Slist, thetalist);
    thetalist = thetalist + pinv(Js) * Vs;
end
% default/fail case
success = false;
% csvwrite('joint_history.csv', history);
fprintf('Max iterations reached, no convergence\n');
end