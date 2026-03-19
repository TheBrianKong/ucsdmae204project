function Jb = J_body(Blist,thetalist)
skew = @(x) [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];
Ad_T = @(R,p) [R zeros(3,3); skew(p)*R R];

Jb = Blist;
T = eye(4);
for i = length(thetalist) - 1: -1: 1   
    omega = Blist(1:3,i+1);
    v= Blist(4:6,i+1);
    S_mat = [skew(omega),v; 0 0 0 0];
    T = T*expm(-S_mat*thetalist(i+1));
    [R,p]= deal(T(1:3,1:3), T(1:3,4));
    Jb(:, i) = Ad_T(R,p) * Blist(:, i);
end
end
