function plotResults(filename)
%
% Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
% Author: Ugo Pattacini <ugo.pattacini@iit.it>

global ha hxd hx
global data
global cnt line

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
global cnt line

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

    hxd=plot3(ha,data.data(j,2),data.data(j,3),data.data(j,4),'ro',...
              'LineWidth',4,'MarkerSize',4);
    drawnow;
end

if ~isempty(k)
    if ~isempty(hx)
        delete(hx);
    end

    hx=plot3(ha,data.data(k,2),data.data(k,3),data.data(k,4),'bo',...
             'LineWidth',4,'MarkerSize',4);
    drawnow;
end

