function endFrame = CreateMarker_RLeg(Marker_Param)
% ----------------------------------------------------------------------- %
% The OpenSim API is a toolkit for musculoskeletal modeling and           %
% simulation. See http://opensim.stanford.edu and the NOTICE file         %
% for more information. OpenSim is developed at Stanford University       %
% and supported by the US National Institutes of Health (U54 GM072970,    %
% R24 HD065690) and by DARPA through the Warrior Web program.             %
%                                                                         %   
% Copyright (c) 2005-2017 Stanford University and the Authors             %
% Author(s): Edith Arnold                                                 %  
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

% setupAndRunIKBatchExample.m                                                 
% Author: Edith Arnold

% This example script runs multiple inverse kinematics trials for the model Subject01. 
% All input files are in the folder ../Matlab/testData/Subject01
% To see the results load the model and ik output in the GUI.

% Pull in the modeling classes straight from the OpenSim distribution
import org.opensim.modeling.*

model           = Marker_Param.model;
s               = Marker_Param.state;
startFrame      = Marker_Param.frame;
PedalPosition   = Marker_Param.pedal;
Time_Start      = Marker_Param.start;
Time_End        = Marker_Param.end;
trc_data_folder = Marker_Param.dir;
markerFile      = Marker_Param.file;

Time_Duration = Time_End - Time_Start;


Start_Point = model.getBodySet().get('Pointer_Toe_R01').getPositionInGround(s);

TimeDuration_IK = 0.01;

if PedalPosition >= 0
    CurrentCoord = model.getCoordinateSet().get('G_Pedal_tilt');
    CurrentCoord.setValue ( s, PedalPosition/100.* 20./180.* pi );
    
    Pointer_Target_1 = model.getBodySet().get('Pointer_Gas').getPositionInGround(s);
else
    CurrentCoord = model.getCoordinateSet().get('B_Pedal_tilt');
    CurrentCoord.setValue ( s, PedalPosition/100.* 30./180.* pi );

    Pointer_Target_1 = model.getBodySet().get('Pointer_Brake').getPositionInGround(s);
end

NumMarkerPnt = cast( ...
    round(Time_Duration/TimeDuration_IK + 1), 'int16');

Reference_Array = zeros(2,4);
Reference_Array(1,1) = Time_Start; Reference_Array(2,1) = Time_End;
Reference_Array(1,2:end)=osimVec3ToArray(Start_Point);
Reference_Array(2,2:end)=osimVec3ToArray(Pointer_Target_1);

NumMarkers = 4;
Marker_Names = {'Marker_toes_r','Marker_heel_r','Marker_tibia_r','Marker_femur_bottom_r'};

Time_Array = zeros( 1, NumMarkerPnt );
Marker_X_Array = zeros ( NumMarkers, NumMarkerPnt );
Marker_Y_Array = zeros ( NumMarkers, NumMarkerPnt );
Marker_Z_Array = zeros ( NumMarkers, NumMarkerPnt );

Time_Array(1,1) = Reference_Array(1,1); Time_Array(1,end) = Reference_Array(2,1);
Marker_X_Array(1,1)   = Reference_Array(1,2); 
Marker_X_Array(1,end) = Reference_Array(2,2);
Marker_Y_Array(1,1)   = Reference_Array(1,3); 
Marker_Y_Array(1,end) = Reference_Array(2,3);
Marker_Z_Array(1,1)   = Reference_Array(1,4); 
Marker_Z_Array(1,end) = Reference_Array(2,4);

for i = 1:cast(NumMarkerPnt,'double') - 2
    Time_Array(1,i+1) = interp1(Reference_Array(:,1),Reference_Array(:,1),...
        Time_Start + TimeDuration_IK * i );
    Marker_X_Array(1,i+1) = interp1(Reference_Array(:,1),Reference_Array(:,2),...
        Time_Start + TimeDuration_IK * i );
    Marker_Y_Array(1,i+1) = interp1(Reference_Array(:,1),Reference_Array(:,3),...
        Time_Start + TimeDuration_IK * i );
    Marker_Z_Array(1,i+1) = interp1(Reference_Array(:,1),Reference_Array(:,4),...
        Time_Start + TimeDuration_IK * i );
end

Reference_Array(1,2:end) = [0.715213,-0.36743,0.0399657]; 
Reference_Array(2,2:end) = Reference_Array(1,2:end);

Marker_X_Array(2,1)   = Reference_Array(1,2); 
Marker_X_Array(2,end) = Reference_Array(2,2);
Marker_Y_Array(2,1)   = Reference_Array(1,3); 
Marker_Y_Array(2,end) = Reference_Array(2,3);
Marker_Z_Array(2,1)   = Reference_Array(1,4); 
Marker_Z_Array(2,end) = Reference_Array(2,4);


for i = 1:cast(NumMarkerPnt,'double') - 2
    Marker_X_Array(2,i+1) = interp1(Reference_Array(:,1),Reference_Array(:,2),...
        Time_Start + TimeDuration_IK * i );
    Marker_Y_Array(2,i+1) = interp1(Reference_Array(:,1),Reference_Array(:,3),...
        Time_Start + TimeDuration_IK * i );
    Marker_Z_Array(2,i+1) = interp1(Reference_Array(:,1),Reference_Array(:,4),...
        Time_Start + TimeDuration_IK * i );
end

Reference_Array(1,2:end) = [0.484428,0.025093,0.0549823]; 
Reference_Array(2,2:end) = Reference_Array(1,2:end);

Marker_X_Array(3,1)   = Reference_Array(1,2); 
Marker_X_Array(3,end) = Reference_Array(2,2);
Marker_Y_Array(3,1)   = Reference_Array(1,3); 
Marker_Y_Array(3,end) = Reference_Array(2,3);
Marker_Z_Array(3,1)   = Reference_Array(1,4); 
Marker_Z_Array(3,end) = Reference_Array(2,4);


for i = 1:cast(NumMarkerPnt,'double') - 2
    Marker_X_Array(3,i+1) = interp1(Reference_Array(:,1),Reference_Array(:,2),...
        Time_Start + TimeDuration_IK * i );
    Marker_Y_Array(3,i+1) = interp1(Reference_Array(:,1),Reference_Array(:,3),...
        Time_Start + TimeDuration_IK * i );
    Marker_Z_Array(3,i+1) = interp1(Reference_Array(:,1),Reference_Array(:,4),...
        Time_Start + TimeDuration_IK * i );
end

Reference_Array(1,2:end) = [0.079607,-0.0936659,0.139444]; 
Reference_Array(2,2:end) = Reference_Array(1,2:end);

Marker_X_Array(4,1)   = Reference_Array(1,2); 
Marker_X_Array(4,end) = Reference_Array(2,2);
Marker_Y_Array(4,1)   = Reference_Array(1,3); 
Marker_Y_Array(4,end) = Reference_Array(2,3);
Marker_Z_Array(4,1)   = Reference_Array(1,4); 
Marker_Z_Array(4,end) = Reference_Array(2,4);


for i = 1:cast(NumMarkerPnt,'double') - 2
    Marker_X_Array(4,i+1) = interp1(Reference_Array(:,1),Reference_Array(:,2),...
        Time_Start + TimeDuration_IK * i );
    Marker_Y_Array(4,i+1) = interp1(Reference_Array(:,1),Reference_Array(:,3),...
        Time_Start + TimeDuration_IK * i );
    Marker_Z_Array(4,i+1) = interp1(Reference_Array(:,1),Reference_Array(:,4),...
        Time_Start + TimeDuration_IK * i );
end



% % Get the name of the file for this trial
% markerFile = trialsForIK(trial).name;

fullpath = ([trc_data_folder '\' markerFile]);

Rate = 1/TimeDuration_IK;

Fid = fopen(fullpath,'wt');
fw_string = sprintf('PathFileType\t%d\t(X/Y/Z)\t%s\n', 4, markerFile);
fprintf (Fid,fw_string);
fw_string = sprintf('DataRate	CameraRate	NumFrames	NumMarkers	Units	OrigDataRate	OrigDataStartFrame	OrigNumFrames\n');
fprintf (Fid,fw_string);
fw_string = sprintf('%d\t%d\t%d\t%d\tm\t%d\t%d\t%d\t\n', Rate, Rate, NumMarkerPnt, NumMarkers, Rate, 1, NumMarkerPnt+1);
fprintf (Fid,fw_string);

fw_string = sprintf('Frame#\tTime');
fprintf (Fid,fw_string);
for i = 1:NumMarkers
    fw_string = sprintf('\t%s\t\t',Marker_Names{i});
    fprintf (Fid,fw_string);    
end
fw_string = sprintf('\n');
fprintf (Fid,fw_string);

fw_string = sprintf('\t');
fprintf (Fid,fw_string);
for i = 1:NumMarkers
    fw_string = sprintf('\tX%d\tY%d\tZ%d',i, i, i);
    fprintf (Fid,fw_string);    
end
fw_string = sprintf('\n');
fprintf (Fid,fw_string);

fw_string = sprintf('\n');
fprintf (Fid,fw_string);

for j = 1:cast(NumMarkerPnt,'double')
    fw_string = sprintf('%d\t%.9f',j - 1 + startFrame, Time_Array(j));
    fprintf (Fid,fw_string);
    for i = 1:NumMarkers
        fw_string = sprintf('\t%.9f\t%.9f\t%.9f',...
            Marker_X_Array(i,j),Marker_Y_Array(i,j),Marker_Z_Array(i,j));
        fprintf (Fid,fw_string);
    end
    fw_string = sprintf('\n');
    fprintf (Fid,fw_string);
end

fclose(Fid);
% Create name of trial from .trc file name

endFrame = startFrame + cast(NumMarkerPnt,'double') - 1;

end


