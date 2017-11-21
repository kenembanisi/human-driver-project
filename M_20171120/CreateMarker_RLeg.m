function [Out_EndPoint,endFrame] = CreateMarker_RLeg(Marker_Param)
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
Time_Start      = Marker_Param.start;
Time_End        = Marker_Param.end;
trc_data_folder = Marker_Param.dir;
Sys_Name        = Marker_Param.Sysname;
markerFile      = Marker_Param.file;
TargetVar_Struct= Marker_Param.TargetVar;
PedalPosition   = TargetVar_Struct.pedal;

TimeCycle_IK    = Marker_Param.cycle;

Time_Duration = round(Time_End - Time_Start,3);



% % Get the name of the file for this trial
% markerFile = trialsForIK(trial).name;

fullpath = ([trc_data_folder '\' Sys_Name '_Master.trc']);


Fid = fopen(fullpath,'rt');

fr_string = fgets(Fid); % PathFileType ...
fr_string = fgets(Fid); % DataRate ...
fr_string = fgets(Fid); % Data of "DataRate ..."
AA = textscan(fr_string,'%d %d %d %d %c %d %d %d');
M.DataRate = AA{1}; M.CameraRate = AA{2};
M.NumFrames = AA{3}; M.NumMarkers = AA{4};
M.Units = char(AA{5});  M.OrigDataRate = AA{6};
M.OrigDataStartFrame = AA{7};  M.OrigNumFrames = AA{8};

fr_string = fgets(Fid); % Frame# ...
AA = textscan(fr_string,'%s','Delimiter','\t');

Marker_Names = cell(M.NumMarkers,1);
for i = 1:M.NumMarkers
    Marker_Names{i,1} = char(AA{1}(i*3));
end

fr_string = fgets(Fid); % X1 Y1 Z1 ...
fr_string = fgets(Fid); % blank
fr_string = fgets(Fid); % Data of "Frame#, Time, X1 ..."
AA = textscan(fr_string,'%f','Delimiter','\t');

Marker_Init_Data = zeros(M.NumMarkers,3);
for i = 1:M.NumMarkers
    Marker_Init_Data(i,1) = AA{1}(i*3);
    Marker_Init_Data(i,2) = AA{1}(i*3+1);
    Marker_Init_Data(i,3) = AA{1}(i*3+2);
end

fclose(Fid);



Repulsions = zeros(3,3);
ReplParams = zeros(3,4);

if PedalPosition >= 0
    for i = 1:size(TargetVar_Struct.TargetBody,1)
        if strfind(TargetVar_Struct.TargetBody{i,1},'G_Pedal')
            break;
        end
    end
    Start_Point = model.getBodySet().get(TargetVar_Struct.TargetSub_Pointer{i,1}).getPositionInGround(s);

    CurrentCoord = model.getCoordinateSet().get(TargetVar_Struct.Target_RelCoord{i,1});
    CurrentCoord.setValue ( s, PedalPosition/100.* 20./180.* pi );
    
    Pointer_Target_1 = model.getBodySet().get(TargetVar_Struct.TargetDes_Pointer{i,1}).getPositionInGround(s);
    
    Repulsions(1,:) = osimVec3ToArray(model.getBodySet().get('B_Pedal_Pot').getPositionInGround(s));
    Repulsions(3,:) = osimVec3ToArray(model.getBodySet().get('G_Pedal_Pot').getPositionInGround(s));

    ReplParams(1,:) = [0.600137494367699 0.015 0.030 0.10];
    ReplParams(3,:) = [0.138634316821442 0.001 0.045 0.01];
    
else
    for i = 1:size(TargetVar_Struct.TargetBody,1)
        if strfind(TargetVar_Struct.TargetBody{i,1},'B_Pedal')
            break;
        end
    end
    Start_Point = model.getBodySet().get(TargetVar_Struct.TargetSub_Pointer{i,1}).getPositionInGround(s);

    CurrentCoord = model.getCoordinateSet().get(TargetVar_Struct.Target_RelCoord{i,1});
    CurrentCoord.setValue ( s, PedalPosition/100.* 30./180.* pi );

    Pointer_Target_1 = model.getBodySet().get(TargetVar_Struct.TargetDes_Pointer{i,1}).getPositionInGround(s);

    Repulsions(1,:) = osimVec3ToArray(model.getBodySet().get('G_Pedal_Pot').getPositionInGround(s));
    Repulsions(3,:) = osimVec3ToArray(model.getBodySet().get('B_Pedal_Pot').getPositionInGround(s));
    
    ReplParams(1,:) = [0.138634316821442 0.015 0.030 0.10];
    ReplParams(3,:) = [0.600137494367699 0.001 0.045 0.01];
    
end

% Marker_Names{1,1}=char(model.getBodySet().get(TargetVar_Struct.TargetSub_Pointer{i,1}).getName);

% NumMarkerPnt = cast( ...
%     round(Time_Duration/TimeDuration_IK + 1), 'int16');
NumMarkerPnt = round(Time_Duration/TimeCycle_IK + 1);

Pot_Field = false;

if ( norm( osimVec3ToArray(Start_Point) - osimVec3ToArray(Pointer_Target_1) ) > 0.02 && Marker_Param.ContForce < 1 )
    Pot_Field = true;

    Rel_Pos = osimVec3ToArray (model.getBodySet.get('Pointer_Gas').getPositionInGround(s))...
        - osimVec3ToArray (model.getBodySet.get('Pointer_Brake').getPositionInGround(s));
    
    if abs(Rel_Pos(3)) > abs(Rel_Pos(2))
        
        p1 = osimVec3ToArray(model.getBodySet.get('B_Pedal').findStationLocationInGround(s, Vec3(0.012645, -0.192043, 0.09804 )));
        p2 = osimVec3ToArray(model.getBodySet.get('G_Pedal').findStationLocationInGround(s, Vec3(-0.087479, -0.092577, -0.042817 )));
        p3 = osimVec3ToArray(model.getBodySet.get('G_Pedal').findStationLocationInGround(s, Vec3(-0.117882, -0.117135, -0.043032  )));
        p4 = osimVec3ToArray(model.getBodySet.get('G_Pedal').findStationLocationInGround(s, Vec3(-0.140837, -0.153889, -0.042488 )));

        [CenterP,r] = SphereFitting_3D(p1,p2,p3,p4);

        Repulsions(2,:) = CenterP;
        ReplParams(2,:) = [r 0.001 0.300 0.20];

    else
        Mid_Pos = ( osimVec3ToArray (model.getBodySet.get('Pointer_Gas').getPositionInGround(s))...
            + osimVec3ToArray (model.getBodySet.get('Pointer_Brake').getPositionInGround(s)) ) / 2.;
        
        Repulsions(2,1) = Mid_Pos(1)+norm(Rel_Pos)*tan(60/180*pi)/2;
        Repulsions(2,2) = Mid_Pos(2);
        Repulsions(2,3) = Mid_Pos(3);
        ReplParams(2,:) = [norm(Rel_Pos)/1000 0.001 0.001 0.002];
        ReplParams(1,1) = ReplParams(1,1) * 0.1;
        ReplParams(3,1) = ReplParams(3,1) * 0.1;
        ReplParams(1,2) = 0.001;
        ReplParams(3,4) = ReplParams(3,4) * 0.1;
        
    end
    
    AuxData = struct();
    AuxData.NumRep = 3;
    AuxData.NumTimSteps = NumMarkerPnt;
    AuxData.RepPos = Repulsions;
    AuxData.RepParams = ReplParams;
    AuxData.StartPos = osimVec3ToArray(Start_Point);
    AuxData.Dest_Pos = osimVec3ToArray(Pointer_Target_1);


    [ax ay az at] = PotField3D_pedals(AuxData);

end


Reference_Array = zeros(2,4);
Reference_Array(1,1) = Time_Start; Reference_Array(2,1) = Time_End;
Reference_Array(1,2:end)=osimVec3ToArray(Start_Point);
Reference_Array(2,2:end)=osimVec3ToArray(Pointer_Target_1);

Time_Array = zeros( 1, NumMarkerPnt );
Marker_X_Array = zeros ( M.NumMarkers, NumMarkerPnt );
Marker_Y_Array = zeros ( M.NumMarkers, NumMarkerPnt );
Marker_Z_Array = zeros ( M.NumMarkers, NumMarkerPnt );

Time_Array(1,1) = Reference_Array(1,1); Time_Array(1,end) = Reference_Array(2,1);
Marker_X_Array(1,1)   = Reference_Array(1,2); 
Marker_X_Array(1,end) = Reference_Array(2,2);
Marker_Y_Array(1,1)   = Reference_Array(1,3); 
Marker_Y_Array(1,end) = Reference_Array(2,3);
Marker_Z_Array(1,1)   = Reference_Array(1,4); 
Marker_Z_Array(1,end) = Reference_Array(2,4);

if Pot_Field
    for i = 1:(NumMarkerPnt - 2)
        Time_Array(1,i+1) = interp1(Reference_Array(:,1),Reference_Array(:,1),...
            Time_Start + TimeCycle_IK * i );
        Marker_X_Array(1,i+1) = ax(i+1);
        Marker_Y_Array(1,i+1) = ay(i+1);
        Marker_Z_Array(1,i+1) = az(i+1);
    end    
else
    for i = 1:(NumMarkerPnt - 2)
        Time_Array(1,i+1) = interp1(Reference_Array(:,1),Reference_Array(:,1),...
            Time_Start + TimeCycle_IK * i );
        Marker_X_Array(1,i+1) = interp1(Reference_Array(:,1),Reference_Array(:,2),...
            Time_Start + TimeCycle_IK * i );
        Marker_Y_Array(1,i+1) = interp1(Reference_Array(:,1),Reference_Array(:,3),...
            Time_Start + TimeCycle_IK * i );
        Marker_Z_Array(1,i+1) = interp1(Reference_Array(:,1),Reference_Array(:,4),...
            Time_Start + TimeCycle_IK * i );
    end
end

EndPoint_in_TaskSpace = zeros(NumMarkerPnt,3);
for i = 1:NumMarkerPnt
    EndPoint_in_TaskSpace(i,1) = Marker_X_Array(1,i);
    EndPoint_in_TaskSpace(i,2) = Marker_Y_Array(1,i);
    EndPoint_in_TaskSpace(i,3) = Marker_Z_Array(1,i);
end

Out_EndPoint = EndPoint_in_TaskSpace;

for i = 2:M.NumMarkers
    Reference_Array(1,2:end) = Marker_Init_Data(i,:); 
    Reference_Array(2,2:end) = Reference_Array(1,2:end);

    Marker_X_Array(i,1)   = Reference_Array(1,2); 
    Marker_X_Array(i,end) = Reference_Array(2,2);
    Marker_Y_Array(i,1)   = Reference_Array(1,3); 
    Marker_Y_Array(i,end) = Reference_Array(2,3);
    Marker_Z_Array(i,1)   = Reference_Array(1,4); 
    Marker_Z_Array(i,end) = Reference_Array(2,4);


    for j = 1:(NumMarkerPnt - 2)
        Marker_X_Array(i,j+1) = interp1(Reference_Array(:,1),Reference_Array(:,2),...
            Time_Start + TimeCycle_IK * j );
        Marker_Y_Array(i,j+1) = interp1(Reference_Array(:,1),Reference_Array(:,3),...
            Time_Start + TimeCycle_IK * j );
        Marker_Z_Array(i,j+1) = interp1(Reference_Array(:,1),Reference_Array(:,4),...
            Time_Start + TimeCycle_IK * j );
    end
    
end



% % Get the name of the file for this trial
% markerFile = trialsForIK(trial).name;

fullpath = ([trc_data_folder '\' markerFile]);

Rate = Marker_Param.freq;

Fid = fopen(fullpath,'wt');
fw_string = sprintf('PathFileType\t%d\t(X/Y/Z)\t%s\n', 4, markerFile);
fprintf (Fid,fw_string);
fw_string = sprintf('DataRate	CameraRate	NumFrames	NumMarkers	Units	OrigDataRate	OrigDataStartFrame	OrigNumFrames\n');
fprintf (Fid,fw_string);
fw_string = sprintf('%d\t%d\t%d\t%d\tm\t%d\t%d\t%d\t\n', Rate, M.CameraRate, NumMarkerPnt, M.NumMarkers, M.OrigDataRate, startFrame, NumMarkerPnt+startFrame-1);
fprintf (Fid,fw_string);

fw_string = sprintf('Frame#\tTime');
fprintf (Fid,fw_string);
for i = 1:M.NumMarkers
    fw_string = sprintf('\t%s\t\t',Marker_Names{i});
    fprintf (Fid,fw_string);    
end
fw_string = sprintf('\n');
fprintf (Fid,fw_string);

fw_string = sprintf('\t');
fprintf (Fid,fw_string);
for i = 1:M.NumMarkers
    fw_string = sprintf('\tX%d\tY%d\tZ%d',i, i, i);
    fprintf (Fid,fw_string);    
end
fw_string = sprintf('\n');
fprintf (Fid,fw_string);

fw_string = sprintf('\n');
fprintf (Fid,fw_string);

for j = 1:NumMarkerPnt
    fw_string = sprintf('%d\t%.9f',j - 1 + startFrame, Time_Array(j));
    fprintf (Fid,fw_string);
    for i = 1:M.NumMarkers
        fw_string = sprintf('\t%.9f\t%.9f\t%.9f',...
            Marker_X_Array(i,j),Marker_Y_Array(i,j),Marker_Z_Array(i,j));
        fprintf (Fid,fw_string);
    end
    fw_string = sprintf('\n');
    fprintf (Fid,fw_string);
end

fclose(Fid);
% Create name of trial from .trc file name

endFrame = startFrame + NumMarkerPnt - 1;

end


