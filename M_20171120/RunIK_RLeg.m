function [Des_Time_Array,Des_Data_Array,Des_Labl_Array] = RunIK_RLeg(IK_Param,Marker_Param)
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

    model = IK_Param.model;
    trc_data_folder = IK_Param.read_dir;
    markerFile = IK_Param.read_file;
    
    results_folder = 'Out_IK';


genericSetupPath = 'AnalyzeSetup';
genericSetupForIK = IK_Param.setup_file;

ikTool = InverseKinematicsTool([genericSetupPath '/' genericSetupForIK]);


  % Check to see if model state is initialized by checking size
    if(model.getWorkingState().getNY() == 0)
       s = model.initSystem();
    else
       s = model.updWorkingState(); 
    end
  
% Tell Tool to use the loaded model
ikTool.setModel(model);

% trialsForIK = dir(fullfile(trc_data_folder, '*.trc'));
% 
% nTrials = size(trialsForIK);

% % Get the name of the file for this trial
% markerFile = trialsForIK(trial).name;

% markerFile = 'two_link_pedal_01.trc';

% Create name of trial from .trc file name
name = regexprep(markerFile,'.trc','');
fullpath = ([trc_data_folder '\' markerFile]);

% Get trc data to determine time range
markerData = MarkerData(fullpath);

% Get initial and intial time 
initial_time = round(markerData.getStartFrameTime(),3);
final_time = round(markerData.getLastFrameTime(),3);

% Setup the ikTool for this trial
ikTool.setName(name);
ikTool.setMarkerDataFileName(fullpath);
ikTool.setStartTime(initial_time);
ikTool.setEndTime(final_time);
IK_output_file = [results_folder '\' name '_ik.mot'];
ikTool.setOutputMotionFileName(IK_output_file);

% Save the settings in a setup file
outfile = ['Setup_IK_' name '.xml'];
ikTool.print([genericSetupPath '\' outfile]);

% fprintf(['Performing IK on cycle # ' num2str(trial) '\n']);

% Run IK
ikTool.run();

% Read IK post data
Des_StoreData = Storage(IK_output_file);

Des_Time = ArrayDouble();
Des_Size = Des_StoreData.getTimeColumn (Des_Time);
%NumMarkerPnt = round(Time_Duration/TimeCycle_IK + 1);
Des_Time_Vector = Des_Time.getAsVector;

Des_Time_Array = osimVectorToArray(Des_Time_Vector);
Des_Data_Array = zeros(size(IK_Param.WholeVarStruct.CoordSet,1),Des_Size);
Des_Labl_Array = cell(size(IK_Param.WholeVarStruct.CoordSet,1),1);


for i = 1:size(IK_Param.WholeVarStruct.CoordSet,1)
    coordvalue = ArrayDouble();
    Des_StoreData.getDataColumn(char(IK_Param.WholeVarStruct.CoordSet{i,1}),coordvalue);
    coordvect = coordvalue.getAsVector;
    Des_Data_Array(i,:) = osimVectorToArray(coordvect);
    Des_Labl_Array{i,1} = char(IK_Param.WholeVarStruct.CoordSet{i,1});
end

Marker_Num = round((Marker_Param.end-Marker_Param.start)/Marker_Param.cycle+1);
if Marker_Num>Des_Size
    Des_Time_Array(end+1) = Des_Time_Array(end) + Marker_Param.cycle;
    Des_Data_Array(:,end+1) = Des_Data_Array(:,end);
end


end