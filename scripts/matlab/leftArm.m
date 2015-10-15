function leftArm
%
% Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
% Author: Ugo Pattacini <ugo.pattacini@iit.it>

global T1 T2 DH q;
global hfig hg_arm hg_target;
global rx;
global init_done;

solver_port='/solver/solution:o';
[rx,fail]=yarpPrepare(solver_port);
if fail
    error('Error: port \"%s\" not found',solver_port);
end

% parameters tripod 1
T1.l=0.09;
T1.theta0=0;
T1.transparency=0.3;
T1.marker_size=4;

% arm
DH=[-0.084, 0.312317, 100*pi/180, 180*pi/180;...
         0,-0.159422,  90*pi/180,  90*pi/180;...
     0.034,        0, -90*pi/180,-100*pi/180;...
         0,   -0.251,  90*pi/180, -90*pi/180;...
         0,        0, -90*pi/180,   0*pi/180;...
         0,   -0.071,-180*pi/180, -90*pi/180];

% tripod 2
T2.l=0.018;
T2.theta0=0;
T2.transparency=0.3;
T2.marker_size=2;

hfig=figure('Name','Arm');
set(hfig,'Toolbar','figure');
hold on; view([1 1 1]); grid;
xlim(0.7*[-1 1]); xlabel('x [m]');
ylim(0.7*[-1 1]); ylabel('y [m]');
zlim(0.7*[-1 1]); zlabel('z [m]');

hax=get(hfig,'CurrentAxes');
set(hax,'DataAspectRatio',[1 1 1]);

lim=axis;
A=max(abs(lim))*0.1;

quiver3(hax,0,0,0,1,0,0,A,'Color','r','Linewidth',2);
quiver3(hax,0,0,0,0,1,0,A,'Color','g','Linewidth',2);
quiver3(hax,0,0,0,0,0,1,A,'Color','b','Linewidth',2);

q=zeros(12,1);
hg_arm=DrawArm;
hg_target=[];

set(hfig,'CloseRequestFcn',@Quit);
init_done=true;



%--------------------------------------------------------------------------
function kinData=ComputeFwKinTripod(T,rho)

kinData.s1=T.l*[cos(T.theta0)        sin(T.theta0)        0]';
kinData.s2=T.l*[cos(T.theta0+2*pi/3) sin(T.theta0+2*pi/3) 0]';
kinData.s3=T.l*[cos(T.theta0+4*pi/3) sin(T.theta0+4*pi/3) 0]';
z=[0 0 1]';

kinData.v1=kinData.s1+rho(1)*z;
kinData.v2=kinData.s2+rho(2)*z;
kinData.v3=kinData.s3+rho(3)*z;
kinData.n=cross(kinData.v2-kinData.v1,kinData.v3-kinData.v1);
kinData.n=kinData.n/norm(kinData.n);

q33=3^1.5*T.l/sqrt(12*(rho(3)^2-(rho(1)+rho(2))*rho(3)+...
                       rho(2)^2-rho(1)*rho(2)+rho(1)^2+27/12*T.l^2));

if q33>=1
    kinData.p1=kinData.v1;
    kinData.p2=kinData.v2;
    kinData.p3=kinData.v3;
    kinData.p=[0 0 rho(1)]';
    kinData.dcm=eye(3,3);
else
    sin_theta=sqrt(1-q33^2);
    ux=-kinData.n(2)/sin_theta;
    uy=kinData.n(1)/sin_theta;
    q11=(1-q33)*ux^2+q33;
    q22=(1-q33)*uy^2+q33;
    q21=(1-q33)*ux*uy;
    q31=-sin_theta*uy;
    q32=sin_theta*ux;
    m1=T.l/q33*(-(1/2)*q11+(3/2)*q22);
    kinData.p=[T.l-m1*q11 -m1*q21 rho(1)-m1*q31]';

    kinData.dcm=[q11 q21 -q31; q21 q22 -q32; q31 q32 q33];
    kinData.p1=kinData.dcm*kinData.s1+kinData.p;
    kinData.p2=kinData.dcm*kinData.s2+kinData.p;
    kinData.p3=kinData.dcm*kinData.s3+kinData.p;
end



%--------------------------------------------------------------------------
function H=ComputeFwKinArm(i)

global DH q;

theta=q(3+i)*pi/180+DH(i,4);
c_theta=cos(theta);
s_theta=sin(theta);
c_alpha=cos(DH(i,3));
s_alpha=sin(DH(i,3));

H=[[c_theta -s_theta*c_alpha  s_theta*s_alpha DH(i,1)*c_theta];...
   [s_theta  c_theta*c_alpha -c_theta*s_alpha DH(i,1)*s_theta];...
   [      0          s_alpha          c_alpha         DH(i,2)];...
   [      0                0                0              1]];



%--------------------------------------------------------------------------
function hg=DrawArm

global T1 T2 q;
global hfig;

set(0,'CurrentFigure',hfig);
hax=get(hfig,'CurrentAxes');
lim=axis(hax);
A=max(abs(lim))*0.1;

kin=ComputeFwKinTripod(T1,q(1:3));

T0=axis2dcm([0 0 1 pi]);
kin.s1=T0*[kin.s1;1];
kin.s2=T0*[kin.s2;1];
kin.s3=T0*[kin.s3;1];
kin.v1=T0*[kin.v1;1];
kin.v2=T0*[kin.v2;1];
kin.v3=T0*[kin.v3;1];
kin.p1=T0*[kin.p1;1];
kin.p2=T0*[kin.p2;1];
kin.p3=T0*[kin.p3;1];
kin.p=T0*[kin.p;1]; kin.p=kin.p(1:3);
kin.n=[T0(1:3,1:3) zeros(3,1); 0 0 0 1]*[kin.n;1];
kin.dcm=T0(1:3,1:3)*kin.dcm;

h1=plot3(hax,[kin.s1(1) kin.v1(1)],[kin.s1(2) kin.v1(2)],...
         [kin.s1(3) kin.v1(3)],...
         'ko-','LineWidth',3,'MarkerSize',T1.marker_size);
h2=plot3(hax,[kin.s2(1) kin.v2(1)],[kin.s2(2) kin.v2(2)],...
         [kin.s2(3) kin.v2(3)],...
         'ko-','LineWidth',3,'MarkerSize',T1.marker_size);
h3=plot3(hax,[kin.s3(1) kin.v3(1)],[kin.s3(2) kin.v3(2)],...
         [kin.s3(3) kin.v3(3)],...
         'ko-','LineWidth',3,'MarkerSize',T1.marker_size);

h4=patch([kin.s1(1) kin.s2(1) kin.s3(1)],...
         [kin.s1(2) kin.s2(2) kin.s3(2)],...
         [kin.s1(3) kin.s2(3) kin.s3(3)],[0.65 0.65 0.65]);
alpha(h4,T1.transparency);

h5=patch([kin.p1(1) kin.p2(1) kin.p3(1)],...
         [kin.p1(2) kin.p2(2) kin.p3(2)],...
         [kin.p1(3) kin.p2(3) kin.p3(3)],'y');
alpha(h5,T1.transparency);

H=[kin.dcm kin.p; 0 0 0 1];
H0=H*eye(4,4);
H1=H0*ComputeFwKinArm(1);
H2=H1*ComputeFwKinArm(2);
H3=H2*ComputeFwKinArm(3);
H4=H3*ComputeFwKinArm(4);
H5=H4*ComputeFwKinArm(5);
H6=H5*ComputeFwKinArm(6);
HN=H6*[ 1 0 0    0;...
        0 1 0    0;...
        0 0 1 0.22;...
        0 0 0    1];

h6=plot3(hax,[kin.p(1) H1(1,4) H2(1,4) H3(1,4) H5(1,4) H6(1,4) HN(1,4)],...
         [kin.p(2) H1(2,4) H2(2,4) H3(2,4) H5(2,4) H6(2,4) HN(2,4)],...
         [kin.p(3) H1(3,4) H2(3,4) H3(3,4) H5(3,4) H6(3,4) HN(3,4)],...
         'k','LineWidth',3);

kin=ComputeFwKinTripod(T2,q(10:12));

hand_ang=20*pi/180; hand_dist=0.07;
TN=axis2dcm([0 1 0 -pi/2+hand_ang]);
TN(1,4)=hand_dist*sin(hand_ang);
TN(3,4)=hand_dist*cos(hand_ang);

Tee=HN*[kin.dcm kin.p; 0 0 0 1]*TN;

kin.s1=HN*[kin.s1;1];
kin.s2=HN*[kin.s2;1];
kin.s3=HN*[kin.s3;1];
kin.v1=HN*[kin.v1;1];
kin.v2=HN*[kin.v2;1];
kin.v3=HN*[kin.v3;1];
kin.p1=HN*[kin.p1;1];
kin.p2=HN*[kin.p2;1];
kin.p3=HN*[kin.p3;1];
kin.p=HN*[kin.p;1]; kin.p=kin.p(1:3);
kin.n=HN(1:3,1:3)*kin.n;

h7=plot3(hax,[kin.s1(1) kin.v1(1)],[kin.s1(2) kin.v1(2)],...
         [kin.s1(3) kin.v1(3)],...
         'ko-','LineWidth',2,'MarkerSize',T2.marker_size);
h8=plot3(hax,[kin.s2(1) kin.v2(1)],[kin.s2(2) kin.v2(2)],...
         [kin.s2(3) kin.v2(3)],...
         'ko-','LineWidth',2,'MarkerSize',T2.marker_size);
h9=plot3(hax,[kin.s3(1) kin.v3(1)],[kin.s3(2) kin.v3(2)],...
         [kin.s3(3) kin.v3(3)],...
         'ko-','LineWidth',2,'MarkerSize',T2.marker_size);

h10=patch([kin.s1(1) kin.s2(1) kin.s3(1)],...
          [kin.s1(2) kin.s2(2) kin.s3(2)],...
          [kin.s1(3) kin.s2(3) kin.s3(3)],[0.65 0.65 0.65]);
alpha(h10,T2.transparency);

h11=patch([kin.p1(1) kin.p2(1) kin.p3(1)],...
          [kin.p1(2) kin.p2(2) kin.p3(2)],...
          [kin.p1(3) kin.p2(3) kin.p3(3)],'y');
alpha(h11,T2.transparency);

h12=quiver3(hax,kin.p(1),kin.p(2),kin.p(3),...
            kin.n(1),kin.n(2),kin.n(3),A,...
            'Color','b','Linewidth',2);
h13=plot3(hax,[kin.p(1) Tee(1,4)],[kin.p(2) Tee(2,4)],...
          [kin.p(3) Tee(3,4)],...
          'k','LineWidth',3);
h14=quiver3(hax,Tee(1,4),Tee(2,4),Tee(3,4),...
            Tee(1,1),Tee(2,1),Tee(3,1),A,...
            'Color','r','Linewidth',2);
h15=quiver3(hax,Tee(1,4),Tee(2,4),Tee(3,4),...
            Tee(1,2),Tee(2,2),Tee(3,2),A,...
            'Color','g','Linewidth',2);
h16=quiver3(hax,Tee(1,4),Tee(2,4),Tee(3,4),...
            Tee(1,3),Tee(2,3),Tee(3,3),A,...
            'Color','b','Linewidth',2);

hg=hggroup;
set(h1,'Parent',hg);
set(h2,'Parent',hg);
set(h3,'Parent',hg);
set(h4,'Parent',hg);
set(h5,'Parent',hg);
set(h6,'Parent',hg);
set(h7,'Parent',hg);
set(h8,'Parent',hg);
set(h9,'Parent',hg);
set(h10,'Parent',hg);
set(h11,'Parent',hg);
set(h12,'Parent',hg);
set(h13,'Parent',hg);
set(h14,'Parent',hg);
set(h15,'Parent',hg);
set(h16,'Parent',hg);



%--------------------------------------------------------------------------
function [t,fail]=yarpPrepare(portName)

t=[];
fail=system('yarp where');
if fail~=0
    fprintf('YARP server seems to be not available\n');
    return;
end

[fail,result]=system(['yarp name query ' portName]);
if fail~=0
    fprintf('%s is unkwnown\n',portName);
    return;
end

idx_ip=strfind(result,'ip');
idx_port=strfind(result,'port');
idx_type=strfind(result,'type');
ip=strtrim(result(idx_ip+2:idx_port-1));
port=str2double(result(idx_port+4:idx_type-1));

t=tcpip(ip,port);
set(t,'InputBufferSize',100000);
set(t,'BytesAvailableFcn',@yarpRxCallback);

fopen(t);
fprintf(t,'CONNECT foo\n');
fprintf(t,'r\n');



%--------------------------------------------------------------------------
function yarpRxCallback(obj,~)

global q;
global hfig;
global hg_arm hg_target;
global init_done;

tline=fgetl(obj);
tline(end)=[];
if ~strcmp(tline,'do')
    if exist('init_done','var')
        if init_done
            [yarpData,status]=str2num(tline); %#ok<ST2NM>
            if status~=0
                q=reshape(yarpData(7:end),size(q));

                set(0,'CurrentFigure',hfig);
                hax=get(hfig,'CurrentAxes');

                if ~isempty(hg_arm)
                    delete(hg_arm)
                end

                if ~isempty(hg_target)
                    delete(hg_target)
                end

                hg_arm=DrawArm;

                hg_target=hggroup;
                lim=axis;
                A=max(abs(lim))*0.1;

                xd=yarpData(1:3);
                ud=yarpData(4:6);
                theta=norm(ud); ud=ud/theta;
                Rd=axis2dcm([ud theta]);

                hp=plot3(hax,xd(1),xd(2),xd(3),...
                         'go','LineWidth',3,'MarkerSize',5);
                hx=quiver3(hax,xd(1),xd(2),xd(3),Rd(1,1),Rd(2,1),Rd(3,1),A,...
                           'Color','r','Linewidth',1);
                hy=quiver3(hax,xd(1),xd(2),xd(3),Rd(1,2),Rd(2,2),Rd(3,2),A,...
                           'Color','g','Linewidth',1);
                hz=quiver3(hax,xd(1),xd(2),xd(3),Rd(1,3),Rd(2,3),Rd(3,3),A,...
                           'Color','b','Linewidth',1);

                set(hp,'Parent',hg_target);
                set(hx,'Parent',hg_target);
                set(hy,'Parent',hg_target);
                set(hz,'Parent',hg_target);

                drawnow;
            end
        end
    end
end



%--------------------------------------------------------------------------
function yarpClose(t)

fclose(t);
delete(t);



%--------------------------------------------------------------------------
function Quit(src,eventdata) %#ok<INUSD>

global rx;

yarpClose(rx);
delete(src);

