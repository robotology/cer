function plotResults(filename,varargin)
% plotResults(filename,orien,do_movie)
% filename the name of the file containing the log
% orien    if false orientation won't be drawn
% do_movie if true a video is recorded
%
% Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
% Author: Ugo Pattacini <ugo.pattacini@iit.it>

global hfig ha hxd hx
global data
global cnt line A
global orien do_movie writer

orien=true;
if nargin>1
    orien=varargin{1};
end

do_movie=false;
if nargin>2
    do_movie=varargin{2};
end

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
fid=fopen(filename);
data=textscan(fid,'%.3f %s %d %.3f %.3f %.3f %.3f %.3f %.3f\n');
fclose(fid);
cnt=0; line=1;

t=timer;
t.Period=0.1;
t.ExecutionMode='fixedRate';
t.TimerFcn=@myCallback;

if do_movie
    writer=VideoWriter('movie','MPEG-4');
    writer.FrameRate=1/t.Period;
    open(writer);
end

start(t);



function myCallback(obj,~,~)

global hfig ha hxd hx
global data
global cnt line A
global orien do_movie writer

cnt=cnt+1;
period=get(obj,'Period');
T=period*cnt;

j=[]; k=[];
while data{1}(line)<T
    type=data{2}(line);
    if strcmp(type,'xd')
        j=line;
    elseif strcmp(type,'x')
        k=line;
    end
    line=line+1;
    if line>length(data{1})
        stop(obj);
        if do_movie
            close(writer);
        end
        delete(obj);
        return;
    end
end

if ~isempty(j)
    if ~isempty(hxd)
        delete(hxd);
    end
    
    x=[data{4}(j) data{5}(j) data{6}(j)];
    u=[data{7}(j) data{8}(j) data{9}(j)]; n=norm(u);
    H=axang2rotm([u/n n]);
    
    hxd=hggroup;
    h1=plot3(ha,x(1),x(2),x(3),'ro','LineWidth',3,'MarkerSize',3);
    set(h1,'Parent',hxd);
    if orien        
        h2=quiver3(ha,x(1),x(2),x(3),H(1,1),H(2,1),H(3,1),A,'r');
        h3=quiver3(ha,x(1),x(2),x(3),H(1,2),H(2,2),H(3,2),A,'g');
        h4=quiver3(ha,x(1),x(2),x(3),H(1,3),H(2,3),H(3,3),A,'b');    
        set(h2,'Parent',hxd);
        set(h3,'Parent',hxd);
        set(h4,'Parent',hxd);
    end
    drawnow;
end

if ~isempty(k)
    if ~isempty(hx)
        delete(hx);
    end
    
    x=[data{4}(k) data{5}(k) data{6}(k)];
    u=[data{7}(k) data{8}(k) data{9}(k)]; n=norm(u);
    H=axang2rotm([u/n n]);

    hx=hggroup;
    h1=plot3(ha,x(1),x(2),x(3),'bo','LineWidth',3,'MarkerSize',3);
    set(h1,'Parent',hx);
    if orien        
        h2=quiver3(ha,x(1),x(2),x(3),H(1,1),H(2,1),H(3,1),A,'r');
        h3=quiver3(ha,x(1),x(2),x(3),H(1,2),H(2,2),H(3,2),A,'g');
        h4=quiver3(ha,x(1),x(2),x(3),H(1,3),H(2,3),H(3,3),A,'b');    
        set(h2,'Parent',hx);
        set(h3,'Parent',hx);
        set(h4,'Parent',hx);
    end
    drawnow;
end

if do_movie
    try
        frame=getframe(hfig);
        writeVideo(writer,frame);
    catch
    end
end    
