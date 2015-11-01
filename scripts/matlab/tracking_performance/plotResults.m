function plotResults(filename)
%
% Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
% Author: Ugo Pattacini <ugo.pattacini@iit.it>

global ha hxd hx
global data
global cnt line A

hfig=figure('Name','Tracking Performance','Color','w');
set(hfig,'Toolbar','figure');
hold on; view([1 1 1]); grid;
xlim(0.7*[-0.1 0.8]); xlabel('x [m]');
ylim(0.7*[-0.5 0.5]); ylabel('y [m]');
zlim(0.7*[-0.5 0.5]); zlabel('z [m]');

ha=get(hfig,'CurrentAxes');
set(ha,'DataAspectRatio',[1 1 1]);

lim=axis;
A=max(abs(lim))*0.1;

quiver3(ha,0,0,0,1,0,0,A,'Color','r','Linewidth',2);
quiver3(ha,0,0,0,0,1,0,A,'Color','g','Linewidth',2);
quiver3(ha,0,0,0,0,0,1,A,'Color','b','Linewidth',2);
A=A*0.75;

hxd=[]; hx=[];
data=importdata(filename);
cnt=0; line=1;

t=timer;
t.Period=0.1;
t.ExecutionMode='fixedRate';
t.TimerFcn=@myCallback;
start(t);



function myCallback(obj,~,~)

global ha hxd hx
global data
global cnt line A

cnt=cnt+1;
period=get(obj,'Period');
T=period*cnt;

j=[]; k=[];
while str2double(data.textdata{line,1})<T
    type=data.textdata{line,2};
    if strcmp(type,'xd')
        j=line;
    elseif strcmp(type,'x')
        k=line;
    end
    line=line+1;
    if line>length(data.data(:,1))
        stop(obj);
        delete(obj);
        break;
    end
end

if ~isempty(j)
    if ~isempty(hxd)
        delete(hxd);
    end
    
    x=data.data(j,2:4);
    u=data.data(j,5:7); n=norm(u);
    H=axis2dcm([u/n n]);
    
    hxd=hggroup;
    h1=plot3(ha,x(1),x(2),x(3),'ro','LineWidth',3,'MarkerSize',3);
    h2=quiver3(ha,x(1),x(2),x(3),H(1,1),H(2,1),H(3,1),A,'r');
    h3=quiver3(ha,x(1),x(2),x(3),H(1,2),H(2,2),H(3,2),A,'g');
    h4=quiver3(ha,x(1),x(2),x(3),H(1,3),H(2,3),H(3,3),A,'b');
    set(h1,'Parent',hxd);
    set(h2,'Parent',hxd);
    set(h3,'Parent',hxd);
    set(h4,'Parent',hxd);
    drawnow;
end

if ~isempty(k)
    if ~isempty(hx)
        delete(hx);
    end
    
    x=data.data(k,2:4);
    u=data.data(k,5:7); n=norm(u);
    H=axis2dcm([u/n n]);   

    hx=hggroup;
    h1=plot3(ha,x(1),x(2),x(3),'bo','LineWidth',3,'MarkerSize',3);
    h2=quiver3(ha,x(1),x(2),x(3),H(1,1),H(2,1),H(3,1),A,'r');
    h3=quiver3(ha,x(1),x(2),x(3),H(1,2),H(2,2),H(3,2),A,'g');
    h4=quiver3(ha,x(1),x(2),x(3),H(1,3),H(2,3),H(3,3),A,'b');
    set(h1,'Parent',hx);
    set(h2,'Parent',hx);
    set(h3,'Parent',hx);
    set(h4,'Parent',hx);   
    drawnow;
end

