function Js= J_space(Slist,thetalist)
%{
space-frame Jacobian.
Slist: joint screw axes in space frame at home position, stored by column
thetalist, joint orientation. 
%}
skew = @(x) [0 -x(3) x(2); x(3) 0 -x(1); -x(2) x(1) 0];
Ad_T = @(R,p) [R zeros(3,3); skew(p)*R R];
T_to_Rp=  @(T) deal(T(1:3,1:3), T(1:3,4));
Js = Slist; % initialize
T = eye(4);
for i = 2: length(thetalist)
    omega = Slist(1:3,i-1); v= Slist(4:6,i-1);
    S_mat = [skew(omega),v; 0 0 0 0];
    T = T * expm(S_mat * thetalist(i - 1));
    [R,p]= T_to_Rp(T);
	Js(:, i) = Ad_T(R,p) * Slist(:, i);
end
end