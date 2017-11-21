%% osimVec3ToArray converts OpenSim Vec3() to a 1x3 Matlab vector
% Input   = OpenSim Vec3()
% Output  = 1x3 Matlab matrix

% ----------------------------------------------------------------------- %
% The OpenSim API is a toolkit for musculoskeletal modeling and           %
% simulation. See http://opensim.stanford.edu and the NOTICE file         %
% for more information. OpenSim is developed at Stanford University       %
% and supported by the US National Institutes of Health (U54 GM072970,    %
% R24 HD065690) and by DARPA through the Warrior Web program.             %
%                                                                         %
% Copyright (c) 2005-2016 Stanford University and the Authors             %
% Author(s): James Dunne                                                  %
%                                                                         %
% Licensed under the Apache License, Version 2.0 (the "License");         %
% you may not use this file except in compliance with the License.        %
% You may obtain a copy of the License at                                 %
% http://www.apache.org/licenses/LICENSE-2.0.                             %
%                                                                         %
% Unless required by applicable law or agreed to in writing, software     %
% distributed under the License is distributed on an "AS IS" BASIS,       %
% WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or         %
% implied. See the License for the specific language governing            %
% permissions and limitations under the License.                          %
% ----------------------------------------------------------------------- %
function vec = osimVectorToArray(p)
% import Java Libraries
import org.opensim.modeling.*
% Check the input type
if strcmp(class(p), 'org.opensim.modeling.Vector')
    vec = zeros(1,size(p));
    for i=0:(size(p)-1)
        vec(i+1) = p.get(i);
    end
else
    error('Incorrect class input. Must be type org.opensim.modeling.Vector') 
end