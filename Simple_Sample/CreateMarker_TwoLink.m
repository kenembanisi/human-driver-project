function endFrame = CreateMarker_TwoLink(Marker_Param)
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

% % move to directory where this subject's files are kept
% subjectDir = uigetdir('testData', 'Select the folder that contains the current subject data');
% 
% subjectDir = '.\';

% % Go to the folder in the subject's folder where .trc files are
% trc_data_folder = uigetdir(subjectDir, 'Select the folder that contains the marker data files in .trc format.');


% % % specify where results will be printed.
% % results_folder = uigetdir(subjectDir, 'Select the folder where the IK Results will be printed.');
% 
% results_folder = 'Out_IK';

% % Get and operate on the files
% % Choose a generic setup file to work from
% [genericSetupForIK,genericSetupPath,FilterIndex] = ...
%     uigetfile('*.xml','Pick the a generic setup file to for this subject/model as a basis for changes.');

% genericSetupPath = 'AnalyzeSetup';
% genericSetupForIK = 'Setup_IK_two_link_04.xml';
% 
% ikTool = InverseKinematicsTool([genericSetupPath genericSetupForIK]);


% % Get the model
% [modelFile,modelFilePath,FilterIndex] = ...
%     uigetfile('*.osim','Pick the the model file to be used.');

% % Load the model and initialize
% model = Model('TwoLinkArmModel.osim');
% s = model.initSystem();
% startFrame = 1;
% PedalPosition = 0.001;
% TimeDration = 0.1;

% % Tell Tool to use the loaded model
% ikTool.setModel(model);
% 
% trialsForIK = dir(fullfile(trc_data_folder, '*.trc'));
% 
% nTrials = size(trialsForIK);

% % Get the name of the file for this trial
% markerFile = trialsForIK(trial).name;
% 
% pedalFile = 'pedal_input_01.mot';

% % Go to the folder in the subject's folder where .trc files are
% trc_data_folder = uigetdir(subjectDir, 'Select the folder that contains the marker data files in .trc format.');
% 
% pedal_data_folder = 'PedalData';
% 
% fullpath = ([pedal_data_folder '\' pedalFile]);
% 
% % Get trc data to determine time range
% PedalData = Storage(fullpath);

% Target_Time = PedalData.getLastTime();
% Pedal_Name = PedalData.getColumnLabels().get(1);


Pointer_Link2 = model.getBodySet().get('Pointer_Link2_Marker_1').getPositionInGround(s);
% Pointer_Wall_0 = model.getBodySet().get('Pointer_Wall').getPositionInGround(s);

% CurrentCoord = model.getCoordinateSet().get('transX');
% CurrentCoord.setValue ( s, PedalData.getLastStateVector().getData.get(0) );

TimeDuration_IK = 0.01;
% Current_Time = s.getTime();

% Target_Time = Time_Start + Time_Duration;
CurrentCoord = model.getCoordinateSet().get('transX');
CurrentCoord.setValue ( s, PedalPosition );

Pointer_Wall_1 = model.getBodySet().get('Pointer_Wall').getPositionInGround(s);

NumMarkerPnt = cast( ...
    round(Time_Duration/TimeDuration_IK + 1), 'int16');

Reference_Array = zeros(2,4);
Reference_Array(1,1) = Time_Start; Reference_Array(2,1) = Time_End;
Reference_Array(1,2:end)=osimVec3ToArray(Pointer_Link2);
Reference_Array(2,2:end)=osimVec3ToArray(Pointer_Wall_1);

NumMarkers = 2;
Marker_Names = {'Link2_Marker_1','Link1_Marker_1'};

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

Reference_Array(1,2:end) = [0.05 0.25 0]; 
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



% % Get the name of the file for this trial
% markerFile = trialsForIK(trial).name;

fullpath = ([trc_data_folder '\' markerFile]);

Rate = 1/TimeDuration_IK;

Fid = fopen(fullpath,'wt');
fw_string = sprintf('PathFileType\t%d\t(X/Y/Z)\t%s\n', 4, markerFile);
fprintf (Fid,fw_string);
fw_string = sprintf('DataRate	CameraRate	NumFrames	NumMarkers	Units	OrigDataRate	OrigDataStartFrame	OrigNumFrames\n');
fprintf (Fid,fw_string);
fw_string = sprintf('%d\t%d\t%d\t%d\tm\t%d\t%d\t%d\t\n', Rate, Rate, NumMarkerPnt, 2, Rate, 1, NumMarkerPnt+1);
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


