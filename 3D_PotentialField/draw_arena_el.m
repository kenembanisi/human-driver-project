function [dps,dpg] = draw_arena_el();

% Routine for drawing transformed arena.

global arena_map arena_r qstart qgoal elliptic_scale RT;

[x,y,z] = sphere;

x1 = x * arena_r;
y1 = y * arena_r;
z1 = z * arena_r;

for i = 1:size(x1,1)
    for j=1:size(x1,2)
        m = [x1(i,j)*elliptic_scale(1) y1(i,j)*elliptic_scale(2) z1(i,j)*elliptic_scale(3)] * RT;
        x1(i,j) = m(1);
        y1(i,j) = m(2);
        z1(i,j) = m(3);
    end
end

surf(x1,y1,z1,'FaceAlpha',0.2,'EdgeAlpha',0.4);  ...
    % ARENA sphere centered at origin with radius arena_r
hold on     % Hold off is in cs458_hw2.m file


for i = 1:length(arena_map)
    x2=x*arena_map{i}(4)+arena_map{i}(1);
    y2=y*arena_map{i}(4)+arena_map{i}(2);
    z2=z*arena_map{i}(4)+arena_map{i}(3);
    
    for k = 1:size(x2,1)
        for j=1:size(x2,2)
            m = [x2(k,j)*elliptic_scale(1) y2(k,j)*elliptic_scale(2) z2(k,j)*elliptic_scale(3)] * RT;
            x2(k,j) = m(1);
            y2(k,j) = m(2);
            z2(k,j) = m(3);
        end
    end
    
    surf(x2,y2,z2,'FaceAlpha',0.6,'EdgeAlpha',0.8);  % sphere centered at (3,-2,0)
end

dps = [0 0 0];
dps(1) = qstart(1)*elliptic_scale(1);
dps(2) = qstart(2)*elliptic_scale(2);
dps(3) = qstart(3)*elliptic_scale(3);

dps = dps * RT;

plot3( dps(1), dps(2), dps(3), '+', 'MarkerFaceColor','g', ...
            'MarkerEdgeColor','g','MarkerSize',8);

dpg = [0 0 0];
dpg(1) = qgoal(1)*elliptic_scale(1);
dpg(2) = qgoal(2)*elliptic_scale(2);
dpg(3) = qgoal(3)*elliptic_scale(3);

dpg = dpg * RT;        
        
plot3( dpg(1), dpg(2), dpg(3), 'x', 'MarkerFaceColor','r', ...
            'MarkerEdgeColor','r','MarkerSize',8);        
        
daspect([1 1 1])    % Axes ratios. Keep them at Original size.

end