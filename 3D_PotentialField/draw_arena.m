function draw_arena();

% Routine for drawing arena.

global arena_map arena_r qstart qgoal qsize;

[x,y,z] = sphere(60);
surf(x*arena_r,y*arena_r,z*arena_r,'FaceAlpha',0.2,'EdgeAlpha',0.4);  ...
    % ARENA sphere centered at origin with radius arena_r
hold on     % Hold off is in cs458_hw2.m file
XX=zeros(size(x,1),size(x,2));
YY=zeros(size(x,1),size(x,2));
ZZ=zeros(size(x,1),size(x,2));
TarP=[0,0,0]; TarR=arena_r;

for k = 1:length(arena_map)

    for i=1:size(x,1)
        for j=1:size(x,2)
            D=sqrt((x(i,j)*arena_map{k}(4)+arena_map{k}(1)-TarP(1))^2.+(y(i,j)*arena_map{k}(4)+arena_map{k}(2)-TarP(2))^2.+(z(i,j)*arena_map{k}(4)+arena_map{k}(3)-TarP(3))^2.);
            if (D<TarR)
                XX(i,j)=x(i,j)*arena_map{k}(4)+arena_map{k}(1);
                YY(i,j)=y(i,j)*arena_map{k}(4)+arena_map{k}(2);
                ZZ(i,j)=z(i,j)*arena_map{k}(4)+arena_map{k}(3);
            else
                XX(i,j)=(x(i,j)*arena_map{k}(4)+arena_map{k}(1)-TarP(1))/D*TarR+TarP(1);
                YY(i,j)=(y(i,j)*arena_map{k}(4)+arena_map{k}(2)-TarP(2))/D*TarR+TarP(2);
                ZZ(i,j)=(z(i,j)*arena_map{k}(4)+arena_map{k}(3)-TarP(3))/D*TarR+TarP(3);
            end
        end
    end    
    surf(XX, ...
         YY, ...
         ZZ, ...
         'FaceAlpha',0.6,'EdgeAlpha',0.8);  % sphere centered at (3,-2,0)
end
plot3( qstart(1), qstart(2), qstart(3), '+', 'MarkerFaceColor','g', ...
            'MarkerEdgeColor','g','MarkerSize',8);

plot3( qgoal(1), qgoal(2), qgoal(3), 'x', 'MarkerFaceColor','r', ...
            'MarkerEdgeColor','r','MarkerSize',8);        
        
daspect([1 1 1])    % Axes ratios. Keep them at Original size.

end