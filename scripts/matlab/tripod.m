function tripod(varargin)
% tripod(true) => generate movie
%
% Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
% Author: Ugo Pattacini <ugo.pattacini@iit.it>

global P s1 s2 s3 z;
global rho;
global xd pitch roll ui_des;
global hfig hg_tripod hg_target;
global tx rx;
global init_done;
global do_movie writer tm;

init_done=false;
do_movie=false;
if nargin>0
    do_movie=varargin{1};
end

solver_port_in='/solver:i';
solver_port_out='/solver:o';

[tx,fail]=yarpPrepare('tx',solver_port_in);
if fail
    error('Error: port \"%s\" not found',solver_port_in);
end
[rx,fail]=yarpPrepare('rx',solver_port_out);
if fail
    yarpClose(tx);
    error('Error: port \"%s\" not found',solver_port_out);
end

% parameters
P.l=0.09;
P.L=0.2;
P.alpha_M=30;
P.transparency=0.3;
P.marker_size=4;

% geometric quantities
s1=P.l*[1 0 0]';
s2=P.l*[-1/2 sqrt(3)/2 0]';
s3=P.l*[-1/2 -sqrt(3)/2 0]';
z=[0 0 1]';
pitch=0;
roll=0;

hfig=figure('Name','Tripod','Color','w');
set(hfig,'Toolbar','figure');
hold on; view([1 1 1]); grid;
xlim(1.5*P.l*[-1 1]); xlabel('x [m]');
ylim(1.5*P.l*[-1 1]); ylabel('y [m]');
zlim(P.L*[-0.1 1.3]); zlabel('z [m]');

hax=get(hfig,'CurrentAxes');
set(hax,'DataAspectRatio',[1 1 1]);

lim=axis;
A=max(abs(lim))*0.1;

quiver3(hax,0,0,0,1,0,0,A,'Color','r','Linewidth',2);
quiver3(hax,0,0,0,0,1,0,A,'Color','g','Linewidth',2);
quiver3(hax,0,0,0,0,0,1,A,'Color','b','Linewidth',2);

% plot circular base
theta=(0:0.01:2*pi)';
points=P.l*[cos(theta) sin(theta) zeros(length(theta),1)];
plot3(hax,points(:,1),points(:,2),points(:,3),'r-');

rho=zeros(3,1);
kinData=ComputeFwKin;
printInfo(kinData);
hg_tripod=DrawTripod(kinData);

xd=[P.L/2 0 0 0];
ax1=patch([s1(1) s2(1) s3(1)],[s1(2) s2(2) s3(2)],...
          [s1(3) s2(3) s3(3)]+xd(1),'y');
alpha(ax1,P.transparency);
ax2=plot3(hax,0,0,xd(1),'ro','MarkerSize',P.marker_size);

hg_target=hggroup('Parent',hax);
set(ax1,'Parent',hg_target);
set(ax2,'Parent',hg_target);

uicontrol(hfig,'Style','Text','Position',[40 300+25 80 15],...
          'String','Target sliders');

uicontrol(hfig,'Style','Text','Position',[0 295 40 30],...
          'String','height [m]');

ui_des{1}=uicontrol(hfig,'Style','Text','Position',[60 300-20+3 40 15],...
                    'String',num2str(xd(1),5));

uicontrol(hfig,'Style','Text','Position',[0 295-20*2 40 30],...
          'String','pitch [deg]');
      
ui_des{2}=uicontrol(hfig,'Style','Text','Position',[60 300-20*3+3 40 15],...
                    'String','0');

uicontrol(hfig,'Style','Text','Position',[0 295-20*4 40 30],...
          'String','roll [deg]');

ui_des{3}=uicontrol(hfig,'Style','Text','Position',[60 300-20*5+3 40 15],...
                    'String','0');

ui_des{4}=uicontrol(hfig,'Style','Text','Position',[12 300-20*7 140 20],...
                    'String','');

uicontrol(hfig,'Style','Slider','Position',[40 300 80 20],...
          'Min',0,'Max',P.L,'Value',xd(1),...
          'Callback',@SelectTarget,'UserData',{1});

uicontrol(hfig,'Style','Slider','Position',[40 300-20*2 80 20],...
          'Min',-P.alpha_M,'Max',P.alpha_M,'Value',0,...
          'Callback',@SelectTarget,'UserData',{2});

uicontrol(hfig,'Style','Slider','Position',[40 300-20*4 80 20],...
          'Min',-P.alpha_M,'Max',P.alpha_M,'Value',0,...
          'Callback',@SelectTarget,'UserData',{3});
      
uicontrol(hfig,'Style','Pushbutton','Position',[40 300-20*6 80 20],...
          'String','Apply Target','Callback',@ApplyTarget);

set(hfig,'CloseRequestFcn',@Quit);

if do_movie
    writer=VideoWriter('movie','MPEG-4');
    writer.FrameRate=20;
    open(writer);
    
    tm=timer('Period',1/writer.FrameRate,'ExecutionMode','fixedRate',...
             'TimerFcn',@SaveFrame,'StopFcn',@StopFcn);
    
    start(tm);
end

init_done=true;



%--------------------------------------------------------------------------
function kinData=ComputeFwKin

global P s1 s2 s3 z;
global rho;

kinData.v1=s1+rho(1)*z;
kinData.v2=s2+rho(2)*z;
kinData.v3=s3+rho(3)*z;
kinData.n=cross(kinData.v2-kinData.v1,kinData.v3-kinData.v1);
kinData.n=kinData.n/norm(kinData.n);

if (rho(1)==rho(2)) && (rho(2)==rho(3))
    kinData.p1=kinData.v1;
    kinData.p2=kinData.v2;
    kinData.p3=kinData.v3;
    kinData.p=[0 0 rho(1)]';
else
    q33=3^1.5*P.l/sqrt(12*(rho(3)^2-(rho(1)+rho(2))*rho(3)+...
                       rho(2)^2-rho(1)*rho(2)+rho(1)^2+27/12*P.l^2));
    sin_theta=sqrt(1-q33^2);
    ux=-kinData.n(2)/sin_theta;
    uy=kinData.n(1)/sin_theta;
    q11=(1-q33)*ux^2+q33;
    q22=(1-q33)*uy^2+q33;
    q21=(1-q33)*ux*uy;
    q31=-sin_theta*uy;
    q32=sin_theta*ux;
    m1=P.l/q33*(-(1/2)*q11+(3/2)*q22);
    kinData.p=[P.l-m1*q11 -m1*q21 rho(1)-m1*q31]';
    
    dcm=[q11 q21 -q31; q21 q22 -q32; q31 q32 q33];
    kinData.p1=dcm*s1+kinData.p;
    kinData.p2=dcm*s2+kinData.p;
    kinData.p3=dcm*s3+kinData.p;
end



%--------------------------------------------------------------------------
function hg=DrawTripod(kinData)

global P s1 s2 s3;
global hfig;

set(0,'CurrentFigure',hfig);
hax=get(hfig,'CurrentAxes');
lim=axis(hax);
A=max(abs(lim))*0.1;

hdl1=plot3(hax,[s1(1) kinData.v1(1)],[s1(2) kinData.v1(2)],[s1(3) kinData.v1(3)],...
           'ko-','LineWidth',3,'MarkerSize',P.marker_size);
hdl2=plot3(hax,[s2(1) kinData.v2(1)],[s2(2) kinData.v2(2)],[s2(3) kinData.v2(3)],...
           'ko-','LineWidth',3,'MarkerSize',P.marker_size);
hdl3=plot3(hax,[s3(1) kinData.v3(1)],[s3(2) kinData.v3(2)],[s3(3) kinData.v3(3)],...
           'ko-','LineWidth',3,'MarkerSize',P.marker_size);

hdl4=patch([kinData.p1(1) kinData.p2(1) kinData.p3(1)],...
           [kinData.p1(2) kinData.p2(2) kinData.p3(2)],...
           [kinData.p1(3) kinData.p2(3) kinData.p3(3)],'y');
       
hdl5=plot3(hax,[kinData.p1(1) kinData.v1(1)],[kinData.p1(2) kinData.v1(2)],...
           [kinData.p1(3) kinData.v1(3)],'k-','LineWidth',1);
hdl6=plot3(hax,[kinData.p2(1) kinData.v2(1)],[kinData.p2(2) kinData.v2(2)],...
           [kinData.p2(3) kinData.v2(3)],'k-','LineWidth',1);
hdl7=plot3(hax,[kinData.p3(1) kinData.v3(1)],[kinData.p3(2) kinData.v3(2)],...
           [kinData.p3(3) kinData.v3(3)],'k-','LineWidth',1);

hdl8=quiver3(hax,kinData.p(1),kinData.p(2),kinData.p(3),...
             kinData.n(1),kinData.n(2),kinData.n(3),A,...
             'Color',[0.65 0.65 0.65],'Linewidth',2);

hg=hggroup;
set(hdl1,'Parent',hg);
set(hdl2,'Parent',hg);
set(hdl3,'Parent',hg);
set(hdl4,'Parent',hg);
set(hdl5,'Parent',hg);
set(hdl6,'Parent',hg);
set(hdl7,'Parent',hg);
set(hdl8,'Parent',hg);



%--------------------------------------------------------------------------
function printInfo(kinData)

global rho;
global hfig;

hax=get(hfig,'CurrentAxes');
title(hax,['\rho'...
      sprintf('=(%5.3f,%5.3f,%5.3f) [m]\np=(%5.3f,%5.3f,%5.3f) [m]\nang=%5.3f [deg]',...
      rho(1),rho(2),rho(3),...
      kinData.p(1),kinData.p(2),kinData.p(3),...
      180/pi*acos(kinData.n(3)))]);



%--------------------------------------------------------------------------
function SelectTarget(src,eventdata) %#ok<INUSD>

global P s1 s2 s3;
global xd pitch roll R ui_des;
global hfig hg_target;

persistent executing;
if isempty(executing);
    executing=false;
end

if executing
    return;
end

executing=true; %#ok<NASGU>

val=get(src,'Value');
UserData=get(src,'UserData');
idx=UserData{1};

if idx==1
    xd(1)=val;
else
    ang=pi/180*val;
    if idx==2
        pitch=ang;
    else
        roll=ang;
    end
end

set(ui_des{idx},'String',num2str(val,5));

R=angle2dcm(0,pitch,roll);
u=rotm2axang(R);
xd(2:4)=u(4)*u(1:3);

rs1=R*s1;
rs2=R*s2;
rs3=R*s3;

set(0,'CurrentFigure',hfig);
hax=get(hfig,'CurrentAxes');

if ~isempty(hg_target)
    delete(hg_target);
end

ax1=patch([rs1(1) rs2(1) rs3(1)],[rs1(2) rs2(2) rs3(2)],...
          [rs1(3) rs2(3) rs3(3)]+xd(1),'y');
alpha(ax1,P.transparency);
ax2=plot3(hax,0,0,xd(1),'ro','MarkerSize',P.marker_size);

hg_target=hggroup('Parent',hax);
set(ax1,'Parent',hg_target);
set(ax2,'Parent',hg_target);
drawnow;

executing=false;



%--------------------------------------------------------------------------
function ApplyTarget(src,evendata) %#ok<INUSD>

global P;
global xd R ui_des;

yarpTxTarget;

tmp=sqrt(3);
m=P.l/R(3,3)*[-0.5*R(1,1)+1.5*R(2,2);...
              R(1,1)+tmp*R(2,1);...
              R(1,1)-tmp*R(2,1)];
rhoIK=xd(1)+m.*[R(3,1);...
                ((tmp*R(3,2)-R(3,1))/2);...
                ((-tmp*R(3,2)-R(3,1))/2)];

set(ui_des{4},'String',sprintf('IK=(%5.3f,%5.3f,%5.3f) [m]',...
    rhoIK(1),rhoIK(2),rhoIK(3)));



%--------------------------------------------------------------------------
function Quit(src,eventdata) %#ok<INUSD>

global tx rx;
global do_movie tm;

yarpClose(tx);
yarpClose(rx);

if do_movie
    stop(tm);
end

delete(src);



%--------------------------------------------------------------------------
function [t,fail]=yarpPrepare(type,portName)

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
if strcmpi(type,'rx')
    set(t,'InputBufferSize',100000);
    set(t,'BytesAvailableFcn',@yarpRxCallback);
end

fopen(t);
fprintf(t,'CONNECT foo\n');

if strcmpi(type,'rx')
    fprintf(t,'r\n');
end



%--------------------------------------------------------------------------
function yarpRxCallback(obj,~)

global rho;
global hg_tripod;
global init_done;

tline=fgetl(obj);
tline(end)=[];
if ~strcmp(tline,'do')
    [yarpData,status]=str2num(tline); %#ok<ST2NM>
    if (status~=0) && exist('init_done','var')
        if init_done
            rho=reshape(yarpData,size(rho));

            if ~isempty(hg_tripod)
                delete(hg_tripod)
            end

            kinData=ComputeFwKin;
            printInfo(kinData);
            hg_tripod=DrawTripod(kinData);
            drawnow;
        end
    end
end



%--------------------------------------------------------------------------
function yarpTxTarget

global xd;
global tx;

fprintf(tx,'do');
fprintf(tx,num2str(xd));



%--------------------------------------------------------------------------
function yarpClose(t)

fclose(t);
delete(t);



%--------------------------------------------------------------------------
function SaveFrame(obj,event,string_arg) %#ok<INUSD>

global hfig;
global writer;

try
    frame=getframe(hfig);
    writeVideo(writer,frame);
catch
end



%--------------------------------------------------------------------------
function StopFcn(obj,event,string_arg) %#ok<INUSD>

global writer;

close(writer);
delete(obj);

