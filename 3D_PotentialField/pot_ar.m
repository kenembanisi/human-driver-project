function gradient = pot_ar(time, rob_pos)

% This function is calculating the gradient of attractive repulsive force
% potential field.

global arena_r;
global arena_map;
global qgoal;
global ObsTh;
global GoalTh;
Kappa = 50;  % Attractive Potential Gain
Nu = 1; % Repulsive Potential Gain

% First find Gradient of Repulsive Potentials
GUrep = [0 0 0];

% Find the repulsive potential exerted by boundary
GUrep_bnd = [0 0 0];
nrp = norm(rob_pos);
D = arena_r - nrp;   % Distance to Boundary (obstacle)
if D<=ObsTh
    Cons = Nu * (1/ObsTh - 1/D) * (1/D^2);  % Constant part
    GUrep_bnd(1) = Cons * (-rob_pos(1)/nrp); % x direction 
    GUrep_bnd(2) = Cons * (-rob_pos(2)/nrp); % y direction
    GUrep_bnd(3) = Cons * (-rob_pos(3)/nrp); % z direction
end

% Then find repulsive potential exerted by obstacles
GUrep_obs = [0 0 0];
for i=1:length(arena_map)
    DtoCenter = sqrt((arena_map{i}(1) - rob_pos(1))^2 + ...
                     (arena_map{i}(2) - rob_pos(2))^2 + ...
                     (arena_map{i}(3) - rob_pos(3))^2 ) ;
    D = DtoCenter - arena_map{i}(4);
    if D<=arena_map{i}(5)
        Cons = Nu * (1/arena_map{i}(5) - 1/D) * (1/D^2);
        GUrep_obs(1) = GUrep_obs(1) + Cons * ((rob_pos(1)-arena_map{i}(1))/DtoCenter); % x direction 
        GUrep_obs(2) = GUrep_obs(2) + Cons * ((rob_pos(2)-arena_map{i}(2))/DtoCenter); % y direction
        GUrep_obs(3) = GUrep_obs(3) + Cons * ((rob_pos(3)-arena_map{i}(3))/DtoCenter); % z direction
    else if D<=arena_map{i}(6)
        Cons = - 0.5 * (arena_map{i}(6) - D);
        GUrep_obs(1) = GUrep_obs(1) + Cons * ((rob_pos(1)-arena_map{i}(1))/DtoCenter); % x direction 
        GUrep_obs(2) = GUrep_obs(2) + Cons * ((rob_pos(2)-arena_map{i}(2))/DtoCenter); % y direction
        GUrep_obs(3) = GUrep_obs(3) + Cons * ((rob_pos(3)-arena_map{i}(3))/DtoCenter); % z direction
    end
end

%Sum Repulsive potential gradients
GUrep = GUrep_bnd + GUrep_obs;

% Then find attractive Potential gradient
GUatt = [0 0 0];
Dummy = [0 0 0];
Dummy(1) = rob_pos(1) - qgoal(1);
Dummy(2) = rob_pos(2) - qgoal(2);
Dummy(3) = rob_pos(3) - qgoal(3);
nrmg = norm(Dummy);
if nrmg <= GoalTh
    GUatt = Kappa * (Dummy);
else
    GUatt = (GoalTh * Kappa / nrmg) * (Dummy);
end

% Finally return total gradient applied on robot... - Gradient will be
% controller input ! Because of gradient decent algorithm.
gradient = [0; 0; 0];
if nrmg > GoalTh*0.04
gradient(1) = (-GUrep(1)) + (-GUatt(1));
gradient(2) = (-GUrep(2)) + (-GUatt(2));
gradient(3) = (-GUrep(3)) + (-GUatt(3));
end

end