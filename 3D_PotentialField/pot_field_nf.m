function [x,y,z,t] = pot_field_nf( qstart, qgoal )

global infinity;

% % A simple hack which blindly goes towards the goal
% samples = norm( qgoal - qstart ) * 10;
% range = linspace(0,1,samples);
% 
% x = qstart(1) + range*(qgoal(1) - qstart(1));
% y = qstart(2) + range*(qgoal(2) - qstart(2));
% z = qstart(3) + range*(qgoal(3) - qstart(3));

rob_pos = qstart;

t0=0; 
tf=infinity; 
tspan=[t0,tf];

options = odeset('Event',@nf_event);

[T,Y] = ode45(@pot_nf,tspan,rob_pos,options);    % TF, Time, ICon, Options

x=Y(:,1);
y=Y(:,2);
z=Y(:,3);
t=T;
end