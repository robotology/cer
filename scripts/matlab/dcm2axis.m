function v=dcm2axis(R)
%
% Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
% Author: Ugo Pattacini <ugo.pattacini@iit.it>

sz=size(R);
assert((sz(1)>=3)&&(sz(2)>=3));

v=zeros(4,1);
v(1)=R(3,2)-R(2,3);
v(2)=R(1,3)-R(3,1);
v(3)=R(2,1)-R(1,2);

r=norm(v);
theta=atan2(0.5*r,0.5*(R(1,1)+R(2,2)+R(3,3)-1));

if r<1e-9
    % if we enter here, then
    % R is symmetric; this can
    % happen only if the rotation
    % angle is 0 (R=I) or 180 degrees
    A=R(1:3,1:3);

    % A=I+sin(theta)*S+(1-cos(theta))*S^2
    % where S is the skew matrix.
    % Given a point x, A*x is the rotated one,
    % hence if Ax=x then x belongs to the rotation
    % axis. We have therefore to find the kernel of
    % the linear application (A-I).      
    [~,~,V]=svd(A-eye(3,3));

    v=V(1:3,3);
    r=norm(v);
end

v=v/r;
v(4)=theta;

end

