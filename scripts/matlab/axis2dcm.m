function R=axis2dcm(v)
%
% Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
% Author: Ugo Pattacini <ugo.pattacini@iit.it>

assert(length(v)>=4);

R=eye(4,4);

theta=v(4);
if theta==0.0
    return;
end

c=cos(theta);
s=sin(theta);
C=1.0-c;

xs =v(1)*s;
ys =v(2)*s;
zs =v(3)*s;
xC =v(1)*C;
yC =v(2)*C;
zC =v(3)*C;
xyC=v(1)*yC;
yzC=v(2)*zC;
zxC=v(3)*xC;

R=[v(1)*xC+c xyC-zs    zxC+ys;...
   xyC+zs    v(2)*yC+c yzC-xs;...
   zxC-ys    yzC+xs    v(3)*zC+c];

end

