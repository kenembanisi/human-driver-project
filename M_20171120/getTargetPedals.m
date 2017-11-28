%#codegen
function Z = getTargetPedals(varargin)
% Mealy State Machine

% A: existence contact force with target interface
% Z: true=force control, false=position control

% y = f(x,u) : 
% all actions are condition actions and 
% outputs are function of state and input 

% define states
S1 = 'B_Pedal';
S2 = 'G_Pedal';
S3 = 'Idle';
S4 = 'Stop';

persistent current_state;
if isempty(current_state)
    current_state = S1;   
end

if nargin == 0
    Z = current_state;
    return;
end

% switch to new state based on the value state register
switch (current_state) 
    
    case S1, % during Moving
        
        % value of output 'Z' depends both on state and inputs
        if (cell2mat(varargin)<0)  % if contact force exists
            Z = current_state;
        else    % if contact force is not measured
            current_state = S2;
            Z = current_state;            
        end
        
    case S2, % during Pressing
        
        if (cell2mat(varargin)<0)  % if contact force exists
            current_state = S1;
            Z = current_state;
        else    % if contact force is not measured
            Z = current_state;
        end
        
%     case S3,
%         
%         if (A)
%             Z = false;            
%             current_state = S2;
%         else
%             Z = true;            
%             current_state = S3;
%         end
%         
%     case S4,
%         
%         if (A)
%             Z = true;
%             current_state = S1;
%         else
%             Z = false;            
%             current_state = S3;
%         end        
        
    otherwise,
        
        Z = false;
end