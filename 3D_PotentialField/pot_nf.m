function grad = pot_nf(time, rob_pos)

global arena_r;
global arena_map;
global qgoal;
global k;

q = [0 0 0];    % Vector
q(1) = rob_pos(1);
q(2) = rob_pos(2);
q(3) = rob_pos(3);

qg = [0 0 0];   % Vector
qg(1) = qgoal(1);
qg(2) = qgoal(2);
qg(3) = qgoal(3);
dqqg = norm(q - qg);   

Ddqqg = (q - qg) / dqqg;    % Vector

B0 = -(norm(q)^2) + arena_r^2;
Bi = [];
Bq = B0;
DB0 = -2 * q;       % Vector
DBi_1 = [];           % Vector Array
DBi_2 = [];
DBi_3 = [];

for i = 1:length(arena_map)
    qi = [0 0 0];
    qi(1) = arena_map{i}(1);
    qi(2) = arena_map{i}(2);
    qi(3) = arena_map{i}(3);
    Bi = [ Bi (norm(q - qi)^2 - (arena_map{i}(4))^2) ];
    Bq = Bq * Bi(i);   % Bq must be ready
    DBi_1 = [DBi_1 (2 * (q(1) - qi(1))) ];
    DBi_2 = [DBi_2 (2 * (q(2) - qi(2))) ];
    DBi_3 = [DBi_3 (2 * (q(3) - qi(3))) ];
end
Bi = [Bi B0];   % Add B0 to the very end of array.
DBi_1 = [DBi_1 DB0(1)];
DBi_2 = [DBi_2 DB0(2)];
DBi_3 = [DBi_3 DB0(3)];

DBq = [0 0 0];  % Vector
for i = 1:(length(arena_map)+1)
    DBi_d = [DBi_1(i) DBi_2(i) DBi_3(i)];
    DBq(1) = DBq(1) + (DBi_d(1) * Bq / Bi(i));   
    DBq(2) = DBq(2) + (DBi_d(2) * Bq / Bi(i));  
    DBq(3) = DBq(3) + (DBi_d(3) * Bq / Bi(i));  
end

p = dqqg^(2*k) + Bq;    %Scalar only

Dp_1k = p^(1/k-1)/k*(2*k*(dqqg)^(2*k-1)*Ddqqg+DBq); % That is a vector

DelNF = ((2*dqqg*Ddqqg)*(p)^(1/k) - (dqqg)^(2)*Dp_1k)/(p)^(2/k); 

grad = [0; 0; 0];
grad(1) = -DelNF(1);
grad(2) = -DelNF(2);
grad(3) = -DelNF(3);

end