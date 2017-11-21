%% osimVec3FromArray converts 1x3 Matlab vector to OpenSim Vec3()
% Input  = 1x3 Matlab matrix
% Output = OpenSim Vec3()

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
function osimMat = osimMatrixFromArray(mat)
% import Java Libraries
import org.opensim.modeling.*
if strcmp(class(mat),'double')
    % Convert the input vector to an OpenSim Vector.
    osimMat = Matrix(size(mat,1),size(mat,2),0.0);
    for i=0:(size(mat,1)-1)
        for j=0:(size(mat,2)-1)
            osimMat.set(i,j,mat(i+1,j+1));
        end
    end
else
    error('Incorrect class input. Must be type double') 
end
