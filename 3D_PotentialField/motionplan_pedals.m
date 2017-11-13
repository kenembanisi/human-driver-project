% CENG786
%
% Homework 2 - Implementing Potential Fields & Navigation Function
%
% This function is where I gathered everything together.
%
% For changing the configuretion space, you can play with variables in the
% init_arena function
%
% First global variables are defined.
%
% Then, Path planning is completed with corresponding functions. 
%
% Finally I draw the arena and generated path.
%
% At the very end, I also showed a simple linear transformation to obtain
% a different workspace other than the sphere-world.
%
% All quantities are in MKS units unless otherwise specified.
%  distances    : m
%  angles       : rad
%  speed        : m/s
%  acceleration : m/s^2

function motionplan_pedals;

% Global Parameters Declarations -----------------
global arena_r;  % Boundaries of the arena: Radius of a sphere @ origin
global arena_map;     % Description of obstacles in the environment
global qstart qgoal qcenter qsize;  % Start and goal configurations
global epsilon_gr;
global epsilon_gr_nf;
global epsilon_goal;
global epsilon;
global ObsTh;   % Q*
global GoalTh;  % Dgoal*
global infinity;  % Large value to be used as 'infinity'
global k;
global ROT_vector;
global ROT_angle;
global elliptic_scale;
global RT;

% Parameter values to be used for the homework ---
arena_r = 2 ;  % Radius ; x, y, z = Always @ origin
arena_map = [];
infinity = 1e100; % 1e5 eeh... 1e6 baya uzar...
epsilon_gr = 2e-3;      % local minima algýlamak için epsilonun izin verilen deðeri
epsilon_gr_nf = 1e-6;
epsilon_goal = 1e-3;
epsilon = 1e-6;
ObsTh = 1;
GoalTh = 3;
k = 1 ;
ROT_vector = [1 1 1];
ROT_angle = pi/4;
RT = rotationmat3D(ROT_angle,ROT_vector);   % Rotation Matrix...
elliptic_scale = [1 2 3];                   % X Y Z axis scales for Sphere to Elliptic Conversion

% Invoking your solutions for the example arena ------------------------
init_arena();

tic
[x_ar, y_ar, z_ar, t_ar] = pot_field_ar( qstart, qgoal );     % Path planning with attractive and repulsive forces
exec_time_ar = toc;

% tic
% [x_nf, y_nf, z_nf, t_nf] = pot_field_nf( qstart, qgoal );     % Path planning with Navigation Function
% exec_time_nf = toc;

x_nf = x_ar;
y_nf = y_ar;
z_nf = z_ar;
t_nf = t_ar;
exec_time_nf = exec_time_ar;

trf_ar = t_ar(length(t_ar));
trf_nf = t_nf(length(t_nf));

%DRAW SPHERICAL CASE
figure(1);
clf;            %Clear Figure...
draw_arena;     %Draw only Arena for visualization, then followed path.
plot3( x_ar, y_ar, z_ar, 'o', 'MarkerFaceColor','b', ...
            'MarkerEdgeColor','b','MarkerSize',3);
plot3( x_nf, y_nf, z_nf, 'o', 'MarkerFaceColor','r', ...
            'MarkerEdgeColor','r','MarkerSize',3);

x_ar_lin = zeros(length(x_ar),1);   % Make the 2D array into 1D (easier for me to handle)
y_ar_lin = zeros(length(x_ar),1);
z_ar_lin = zeros(length(x_ar),1);
for i = 1:size(x_ar,1)
    for j=1:size(x_ar,2)
        x_ar_lin((i-1)*size(x_ar,2) + j)=x_ar(i,j);% Make the 2D array into 1D (easier for me to handle)
        y_ar_lin((i-1)*size(x_ar,2) + j)=y_ar(i,j);
        z_ar_lin((i-1)*size(x_ar,2) + j)=z_ar(i,j);
    end
end 

x_nf_lin = zeros(length(x_nf),1);
y_nf_lin = zeros(length(x_nf),1);
z_nf_lin = zeros(length(x_nf),1);        
for i = 1:size(x_nf,1)
    for j=1:size(x_nf,2)
        x_nf_lin((i-1)*size(x_nf,2) + j)=x_nf(i,j);
        y_nf_lin((i-1)*size(x_nf,2) + j)=y_nf(i,j);
        z_nf_lin((i-1)*size(x_nf,2) + j)=z_nf(i,j);
    end
end
        
pathl_ar=0;   % Calculate Path Lenghts, One more thing to compare...     
for i=1:(length(x_ar_lin)-1)
    pathl_ar = pathl_ar + sqrt((x_ar_lin(i)-x_ar_lin(i+1))^2 + ...
                                (y_ar_lin(i)-y_ar_lin(i+1))^2 + ...
                                (z_ar_lin(i)-z_ar_lin(i+1))^2);
end
pathl_nf=0;        
for i=1:(length(x_nf_lin)-1)
    pathl_nf = pathl_nf + sqrt((x_nf_lin(i)-x_nf_lin(i+1))^2 + ...
                                (y_nf_lin(i)-y_nf_lin(i+1))^2 + ...
                                (z_nf_lin(i)-z_nf_lin(i+1))^2);
end
        
str = sprintf(['Navigation with Potential Functions - Spherical World \n'...
    'Excution Time of Algorithm = ',num2str(exec_time_ar),'s Att-Rep, ',num2str(exec_time_nf),'s NavFunc \n'...
    'Time ellapsed for Robot = ',num2str(trf_ar),'s Att-Rep, ',num2str(trf_nf),'s NavFunc \n'...
    'Approximate Path Lengths = ',num2str(pathl_ar),'m Arr-Rep, ',num2str(pathl_nf),'m NavFunc \n'...
    'Initial Position = [',num2str(qstart(1)),' ',num2str(qstart(2)),' ',num2str(qstart(3)),'], Goal Position = [',num2str(qgoal(1)),' ',num2str(qgoal(2)),' ',num2str(qgoal(3)),'] \n'...
    'Parameters of NavFunc => k=',num2str(k), '\n'...
    'Parameters of Att-Rep => Goal Threshold=',num2str(GoalTh),' Obstacle Threshold=',num2str(ObsTh)]);
        
title(str);
xlabel('X Coordinate (m)');
ylabel('Y Coordinate (m)');
zlabel('Z Coordinate (m)');
grid on;
%colorbar;   % colorbar gives an idea about z position of obstacles
hold off;   % starts in draw arena5

% %DRAW FOR ELLIPTICAL SCALED VERSION 
% figure(2);
% clf;            %Clear Figure...
% [modif_s,modif_g] = draw_arena_el;     %Draw only Arena for visualization, then followed path.
% 
% x_ar_lin = zeros(1,1);        
% y_ar_lin = zeros(1,1);
% z_ar_lin = zeros(1,1);
% x_ar_lin = zeros(length(x_ar),1);   % Make the 2D array into 1D (easier for me to handle)
% y_ar_lin = zeros(length(x_ar),1);
% z_ar_lin = zeros(length(x_ar),1);
% for i = 1:size(x_ar,1)
%     for j=1:size(x_ar,2)
%         m = [x_ar(i,j)*elliptic_scale(1) y_ar(i,j)*elliptic_scale(2) z_ar(i,j)*elliptic_scale(3)] * RT;
%         x_ar(i,j) = m(1);
%         x_ar_lin((i-1)*size(x_ar,2) + j)=x_ar(i,j);% Make the 2D array into 1D (easier for me to handle)
%         y_ar(i,j) = m(2);
%         y_ar_lin((i-1)*size(x_ar,2) + j)=y_ar(i,j);
%         z_ar(i,j) = m(3);
%         z_ar_lin((i-1)*size(x_ar,2) + j)=z_ar(i,j);
%     end
% end
% plot3( x_ar, y_ar, z_ar, 'o', 'MarkerFaceColor','b', ...
%             'MarkerEdgeColor','b','MarkerSize',3);
%         
% x_nf_lin = zeros(length(x_nf),1);
% y_nf_lin = zeros(length(x_nf),1);
% z_nf_lin = zeros(length(x_nf),1);        
% for i = 1:size(x_nf,1)
%     for j=1:size(x_nf,2)
%         m = [x_nf(i,j)*elliptic_scale(1) y_nf(i,j)*elliptic_scale(2) z_nf(i,j)*elliptic_scale(3)] * RT;
%         x_nf(i,j) = m(1);
%         x_nf_lin((i-1)*size(x_nf,2) + j)=x_nf(i,j);
%         y_nf(i,j) = m(2);
%         y_nf_lin((i-1)*size(x_nf,2) + j)=y_nf(i,j);
%         z_nf(i,j) = m(3);
%         z_nf_lin((i-1)*size(x_nf,2) + j)=z_nf(i,j);
%     end
% end
% plot3( x_nf, y_nf, z_nf, 'o', 'MarkerFaceColor','r', ...
%             'MarkerEdgeColor','r','MarkerSize',3);
%         
% pathl_ar=0;   % Calculate Path Lenghts, One more thing to compare...     
% for i=1:(length(x_ar_lin)-1)
%     pathl_ar = pathl_ar + sqrt((x_ar_lin(i)-x_ar_lin(i+1))^2 + ...
%                                 (y_ar_lin(i)-y_ar_lin(i+1))^2 + ...
%                                 (z_ar_lin(i)-z_ar_lin(i+1))^2);
% end
% pathl_nf=0;        
% for i=1:(length(x_nf_lin)-1)
%     pathl_nf = pathl_nf + sqrt((x_nf_lin(i)-x_nf_lin(i+1))^2 + ...
%                                 (y_nf_lin(i)-y_nf_lin(i+1))^2 + ...
%                                 (z_nf_lin(i)-z_nf_lin(i+1))^2);
% end
%         
% str = sprintf(['Navigation with Potential Functions - Non-Spherical (Ellipsoid) World \n'...
% 'Excution Time of Algorithm = ',num2str(exec_time_ar),'s Att-Rep, ',num2str(exec_time_nf),'s NavFunc \n'...
% 'Time ellapsed for Robot = ',num2str(trf_ar),'s Att-Rep, ',num2str(trf_nf),'s NavFunc \n'...
% 'Approximate Path Lengths = ',num2str(pathl_ar),'m Arr-Rep, ',num2str(pathl_nf),'m NavFunc \n'...
% 'Initial Position = [',num2str(modif_s(1)),' ',num2str(modif_s(2)),' ',num2str(modif_s(3)),'], Goal Position = [',num2str(modif_g(1)),' ',num2str(modif_g(2)),' ',num2str(modif_g(3)),'] \n'...
% 'Parameters of NavFunc => k=',num2str(k), '\n'...
% 'Parameters of Att-Rep => Goal Threshold=',num2str(GoalTh),' Obstacle Threshold=',num2str(ObsTh),'\n'...
% 'Rotation Vector (from origin) = [',num2str(ROT_vector(1)),' ',num2str(ROT_vector(2)),' ',num2str(ROT_vector(3)),'] , Rotation Angle = ',num2str(ROT_angle),'rad \n'...
% 'Sphere to Ellipsoid scaling in x, y, z axes = [',num2str(elliptic_scale(1)),' ',num2str(elliptic_scale(2)),' ',num2str(elliptic_scale(3)),']' ]);
% 
% title(str);
% xlabel('X Coordinate (m)');
% ylabel('Y Coordinate (m)');
% zlabel('Z Coordinate (m)');
% grid on;
% colorbar;   % colorbar gives an idea about z position of obstacles
% hold off;   % starts in draw arena5


function init_arena;    % Also modify k in this function too. For each arena
% Optimal k parameter differs...
global arena_map qstart qgoal qcenter qsize k ObsTh GoalTh;

arena_map = [];

% %%%%%%%%CONFIG 1 A Simple Case with 1 Obstacle
% k=2;
% ObsTh = 1;
% GoalTh = 3;
% arena_map{1} = [ 0 0 0 3 ];    % x y z r
% qstart = [-9.5 0.01 0];   % x y z
% qgoal = [8 0 0];      % x y z

%%%%%%%%%CONFIG 2 Multiple Obstacles, No Local Minima
k=.0002;
qsize = 10.0;
ObsTh = .001 * qsize;
GoalTh = .15 * qsize;
arena_map{1} = [ 0.9790 -0.2662 0.1814 0.1365 0.01 0.03 ];  % G pedal x y z r ObsTh1
arena_map{2} = [ 1.8401 -0.2886 0.0031 0.9481 0.014 0.045 ];  % B pedal x y z r
arena_map{3} = [ 2.169  -1.5066 0.4902 1.8794 0.02 0.245 ];  % middle  x y z r
% arena_map{4} = [  0.834207 -0.236235 0.096253 0.07 ];    % x y z r
% arena_map{5} = [  0.834207 -0.236235 0.136253 0.07 ];    % x y z r
qstart = [ 0.862072 -0.177591 0.1845];   % x y z
qgoal = [0.889479 -0.171024 0.024379];      % x y z

qcenter = (qstart + qgoal)/2;

arena_map{1} = (arena_map{1} - [qcenter 0 0 0]) * qsize;
arena_map{2} = (arena_map{2} - [qcenter 0 0 0]) * qsize;
arena_map{3} = (arena_map{3} - [qcenter 0 0 0]) * qsize;
% arena_map{4} = (arena_map{4} - [qcenter 0]) * qsize;
% arena_map{5} = (arena_map{5} - [qcenter 0]) * qsize;
qstart = (qstart - qcenter) * qsize;
qgoal = (qgoal - qcenter) * qsize;

% %%%%%%%%%CONFIG 3 Multiple Obstacles, With Local Minima
% 
% arena_map{1} = [ 0 -1  0 0.5 ];    % x y z r
% arena_map{2} = [ 0  0  1 0.5 ];    % x y z r
% arena_map{3} = [ 0  1  0 0.5 ];    % x y z r
% arena_map{4} = [ 0  0 -1 0.5 ];    % x y z r
% 
% qstart = [-9.5 0.01 0.01];   % x y z
% qgoal = [8 0 0];      % x y z

% %%%%%%%%%CONFIG 4 Multiple Obstacles, With Local Minima
% arena_map{1} = [ 0 -1  0 0.5 ];    % x y z r
% arena_map{2} = [ 0  0  1 0.5 ];    % x y z r
% arena_map{3} = [ 0  1  0 0.5 ];    % x y z r
% arena_map{4} = [ 0  0 -1 0.5 ];    % x y z r
% 
% arena_map{5} = [ 0  6 0 3 ];    % x y z r
% arena_map{6} = [ 0  0 6 3 ];    % x y z r
% arena_map{7} = [ 0  0 -6 3 ];    % x y z r
% arena_map{8} = [ 0  -6 0 3 ];    % x y z r
% arena_map{9} = [ 0  3 3 1 ];    % x y z r
% arena_map{10} = [ 0  -3 3 1 ];    % x y z r
% arena_map{11} = [ 0  3 -3 1 ];    % x y z r
% arena_map{12} = [ 0  -3 -3 1 ];    % x y z r
% qstart = [-9.5 0.01 0.01];   % x y z
% qgoal = [8 0 0];      % x y z

%%%%%%%%%CONFIG 5 
