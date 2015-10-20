function table_stability(filename)
%
% Copyright (C) 2015 iCub Facility - Istituto Italiano di Tecnologia
% Author: Ugo Pattacini <ugo.pattacini@iit.it>

hfig=figure('Name','Table Stability','Color','w');
set(hfig,'Toolbar','figure');
hold on; view([1 1 1]); grid;
xlim(0.7*[-0.5 1.5]); xlabel('x [m]');
ylim(0.7*[-1.5 1.5]); ylabel('y [m]');
zlim(0.7*[-1 1]); zlabel('z [m]');

hax=get(hfig,'CurrentAxes');
set(hax,'DataAspectRatio',[1 1 1]);

lim=axis;
A=max(abs(lim))*0.1;

quiver3(hax,0,0,0,1,0,0,A,'Color','r','Linewidth',2);
quiver3(hax,0,0,0,0,1,0,A,'Color','g','Linewidth',2);
quiver3(hax,0,0,0,0,0,1,A,'Color','b','Linewidth',2);

floor_z=-0.63;

% casters
c1=[0.078  0.09];
c2=[-0.244 0.0];
c3=[c1(1) -c1(2)];

caster_reduction=0.023;

r=norm(c1)-caster_reduction;
theta=atan2(c1(2),c1(1));
c1=r*[cos(theta) sin(theta)];

r=norm(c2)-caster_reduction;
theta=atan2(c2(2),c2(1));
c2=r*[cos(theta) sin(theta)];

r=norm(c3)-caster_reduction;
theta=atan2(c3(2),c3(1));
c3=r*[cos(theta) sin(theta)];

% support polygon
hp=patch([c1(1) c3(1) -0.074 c2(1) -0.074],...
         [c1(2) c3(2) -0.17 c2(2) 0.17],...
         floor_z*ones(1,5),[0.65 0.65 0.65]);
alpha(hp,0.75);

data=dlmread(filename);
sz=size(data);

for i=1:sz(1)
    c='bo';
    if data(i,end)<=0
        c='ro';
    end
    plot3(data(i,1),data(i,2),data(i,3),c,...
          'LineWidth',2,'MarkerSize',4);
    plot3([data(i,1) data(i,4)],[data(i,2) data(i,5)],...
          [data(i,3) data(i,6)],'k-');
    plot3(data(i,9),data(i,10),floor_z,'kd',...
          'MarkerSize',2);     
end

