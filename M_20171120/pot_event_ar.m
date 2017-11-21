function [value,isterminal,direction] = pot_event_ar(t,rob_pos)

% This event is checks whther goal state is reached or a local minima is
% observed. So that ODE solution can termnate before the end of given time span. 

global epsilon_gr;
global epsilon_goal;
global epsilon;
global qgoal;

% find whether gradient is almost 0 in any direction => i.e. local minima

gradient = pot_ar(t,rob_pos);

Dummy = [0 0 0];
Dummy(1) = rob_pos(1)-qgoal(1);
Dummy(2) = rob_pos(2)-qgoal(2);
Dummy(3) = rob_pos(3)-qgoal(3);
m=norm(Dummy);

 value = [ (m<=epsilon_goal)*1  (norm(gradient)>epsilon_gr)*1];  % ()*1 is necessary for MATLAB logical operations... Don't ask me why... Because... Thats why...

isterminal = [ 1 1 ];   % Stop the integration
direction = [ 0 0 ];   % any direction


end