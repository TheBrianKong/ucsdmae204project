function x = unskew(X)
%UNSKEW get 3x1 vector back from skew symmetric matrix X
x = [X(3,2); X(1,3); X(2,1)];
end

