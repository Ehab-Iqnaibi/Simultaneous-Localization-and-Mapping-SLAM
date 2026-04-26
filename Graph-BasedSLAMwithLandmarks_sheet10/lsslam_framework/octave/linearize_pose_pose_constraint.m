% Compute the error of a pose-pose constraint
% x1 3x1 vector (x,y,theta) of the first robot pose
% x2 3x1 vector (x,y,theta) of the second robot pose
% z 3x1 vector (x,y,theta) of the measurement
%
% You may use the functions v2t() and t2v() to compute
% a Homogeneous matrix out of a (x, y, theta) vector
% for computing the error.
%
% Output
% e 3x1 error of the constraint
% A 3x3 Jacobian wrt x1
% B 3x3 Jacobian wrt x2
function [e, A, B] = linearize_pose_pose_constraint(x1, x2, z)

  % TODO compute the error and the Jacobians of the error
  X1 = v2t(x1);
  X2 = v2t(x2);
  Z = v2t(z);

  e = t2v(pinv(Z) * (pinv(X1) * X2));

  A = [-cos(x1(3)+z(3)) -sin(x1(3)+z(3)) (x1(1)-x2(1))*sin(x1(3)+z(3)) - (x1(2)-x2(2))*cos(x1(3)+z(3))
        sin(x1(3)+z(3)) -cos(x1(3)+z(3)) (x1(1)-x2(1))*cos(x1(3)+z(3)) + (x1(2)-x2(2))*sin(x1(3)+z(3))
        0                0                -1                                                          ];

  B = [ cos(x1(3)+z(3)) sin(x1(3)+z(3)) 0
       -sin(x1(3)+z(3)) cos(x1(3)+z(3)) 0
        0               0               1];


end;
