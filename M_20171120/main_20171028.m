% ----------------------------------------------------------------------- %
% main_YYYYMMDD.m
%
% Author: Hideyuki Kimpara, Worcester Polytechnic Institute
%
% This code uses the Matlab interface to the OpenSim API to run a trial
% to connect simulink simulator and OpenSim analysis. 
% This example borrows from other Matlab and C++ examples avaiable from
% the OpenSim project written by several people including, but not limited
% to: Brian Umberger and OpenSim team members at Stanford University.
% ----------------------------------------------------------------------- %
clearvars
clear

% Node and timing information
System_StartTim  = 0.00;
System_End_Time  = 000.90;
System_Frequency = fix(1000.);

if System_Frequency < 1
    System_Frequency = 1.;
end



% Import the OpenSim modeling classes
import org.opensim.modeling.*

if str2num(char(org.opensim.modeling.opensimCommon.GetVersion())) < 4.0
    disp(['Please install OpenSim 4.0 or greater than 4.0']);
    exit(-1);
end

pause(1);


% Load the .osim file that contain Model data
[file_input, pathname] = uigetfile({'*.osim', 'OpenSim Model Files (*.osim)'}, ...
                         'Select the OpenSim model file','MultiSelect', 'off');

[pathstr,Sys_Name,ext] = fileparts(strcat(pathname,file_input));

TargetVar_Struct = InitTargetStruct (pathname,Sys_Name);

osimModel = TargetVar_Struct.Model;

% Add Analyses to the model: ForceReporter and BodyKinematics 
% aActuation = Actuation(osimModel);
% aBodyKinematics = BodyKinematics(osimModel);
% aForceReporter = ForceReporter(osimModel);
% aJointReaction = JointReaction(osimModel);
% aKinematics = Kinematics(osimModel);
aStatesReporter = StatesReporter(osimModel);
% osimModel.addAnalysis(aActuation);
% osimModel.addAnalysis(aBodyKinematics);
% osimModel.addAnalysis(aForceReporter);
% osimModel.addAnalysis(aJointReaction);
% osimModel.addAnalysis(aKinematics);
osimModel.addAnalysis(aStatesReporter);


% Check to see if model state is initialized by checking size
if(osimModel.getWorkingState().getNY() == 0)
   osimState = osimModel.initSystem();
else
   osimState = osimModel.updWorkingState(); 
end

%# Set the initial states of the model
editableCoordSet = osimModel.updCoordinateSet();

% Arrange the initial guess by nodes and states
for i=1:TargetVar_Struct.Number
    editableCoordSet.get(TargetVar_Struct.Cent_cor_name{1,i}).setValue(osimState, TargetVar_Struct.Q_ini(1,i));
    editableCoordSet.get(TargetVar_Struct.Cent_cor_name{1,i}).setSpeedValue(osimState, TargetVar_Struct.U_ini(1,i));
end




% StateVars of whole model in osimModel definition
WholeVarStruct = InitWholeVarStruct (osimModel, osimState);


pedalFile = [Sys_Name,'_pedal.mot'];

% % Go to the folder in the subject's folder where .trc files are
% trc_data_folder = uigetdir(subjectDir, 'Select the folder that contains the marker data files in .trc format.');

pedal_data_folder = 'PedalData';

fullpath = ([pedal_data_folder '\' pedalFile]);

PedalData = getPedalData (fullpath);

fileoutpath = ['Monitor_Model.osim'];
osimModel.print(fileoutpath);

% Prepare temporary export model file for debug
fileoutpath = [Sys_Name,'_Monitor_01.osim'];
Frame = 1;

stateStorage = aStatesReporter.getStatesStorage();

UpdatedStateVals = zeros(size(WholeVarStruct.StateLabl,1),1);
UpdatedExtForces.Target = char([]);
UpdatedExtForces.Force = zeros(6,1);

State_Event = 'Move_to_Brake';

% start a timer
tic;


for ii = 2:size(PedalData,2)
   % osimState.setTime(Target_Time.get(ii-2));
    
    System_StartTim  = osimState.getTime;
    System_End_Time  = PedalData(1,ii);
    
    TargetVar_Struct.pedal = PedalData(2,ii);
    if TargetVar_Struct.pedal >= 0
        TargetVar_Struct.Target_current = 'G_Pedal';
    else
        TargetVar_Struct.Target_current = 'B_Pedal';
    end

    if ~strcmp(UpdatedExtForces.Target,TargetVar_Struct.Target_current)
        UpdatedExtForces.Force = zeros(6,1);
    end
    
    Force_Control = mlhdlc_fsm_mealy(sum(UpdatedExtForces.Force));
    
    Marker_Param.model = osimModel;
    Marker_Param.state = osimState;
    Marker_Param.frame = Frame;
    Marker_Param.start = System_StartTim;
    Marker_Param.end   = System_End_Time;
    Marker_Param.dir   = 'MarkerData';
    Marker_Param.Sysname = Sys_Name;
    Marker_Param.file  = sprintf('%s_%03d.trc', Sys_Name, ii-1);
    Marker_Param.TargetVar = TargetVar_Struct;

    Next = CreateMarker_RLeg(Marker_Param);
    Frame = Next;
    
    IK_Param.model     = osimModel;
    IK_Param.read_dir  = Marker_Param.dir;
    IK_Param.read_file = Marker_Param.file;
    IK_Param.setup_dir = 'AnalyzeSetup';
    IK_Param.setup_file= [Sys_Name,'_IK_Setup_Master.xml'];
    IK_Param.out_dir   = 'Out_IK';
   
    
    IK_outfile = RunIK_RLeg(IK_Param);

    motfilepath = IK_outfile;
    Des_StoreData = Storage(motfilepath);
    
    Des_Time = ArrayDouble();
    Des_Size = Des_StoreData.getTimeColumn (Des_Time);

    Des_Time_Vector = Des_Time.getAsVector;

    Des_Time_Array = osimVectorToArray(Des_Time_Vector);
    Des_Data_Array = zeros(size(WholeVarStruct.CoordSet,1),Des_Size);
    Des_Labl_Array = cell(size(WholeVarStruct.CoordSet,1),1);

    for i = 1:size(WholeVarStruct.CoordSet,1)
        coordvalue = ArrayDouble();
        Des_StoreData.getDataColumn(char(WholeVarStruct.CoordSet{i,1}),coordvalue);
        coordvect = coordvalue.getAsVector;
        Des_Data_Array(i,:) = osimVectorToArray(coordvect);
        Des_Labl_Array{i,1} = char(WholeVarStruct.CoordSet{i,1});
    end
    
    
    System_timestep = 1./System_Frequency;

    System_Num_step = fix((System_End_Time-System_StartTim)*System_Frequency);


    
    Control_Func.time = zeros(1, System_Num_step*2+1);
    Control_Func.data = zeros(TargetVar_Struct.Number, System_Num_step*2+1);
    Control_Func.name = TargetVar_Struct.Cent_act_name;
    Control_Func.time(1) = System_StartTim;
    for i=1:System_Num_step
        Control_Func.time(i*2)   = System_StartTim + System_timestep * (i-1) + System_timestep*0.1;
        Control_Func.time(i*2+1) = System_StartTim + System_timestep * (i);
    end
    
    
    Des_Labels = Des_StoreData.getColumnLabels;


    for i = 1:System_Num_step
        time_start = System_StartTim + System_timestep * (i-1);
        time_end = System_StartTim + System_timestep * (i);

        if Des_Size
            j = 0.;
            if time_end < Des_Time_Array(1,1)
                desTime_adr = 1;
            elseif time_end > Des_Time_Array(1,Des_Size)
                desTime_adr = Des_Size;
            else
                desTime_adr = Des_Size;
                for j = 1:Des_Size
                    if time_end < Des_Time_Array(1,j)
                        desTime_adr = j;
                        break;
                    end
                end
            end

            for j = 1:TargetVar_Struct.Number
                for k = 1:size(WholeVarStruct.CoordSet,1)
                    if strcmp(TargetVar_Struct.Cent_cor_name{1,j},Des_Labl_Array{k,1})
                        TargetVar_Struct.Q_des(1,j) = Des_Data_Array(k,desTime_adr)/180*pi;
                        if desTime_adr > 1
                            TargetVar_Struct.U_des(1,j) = (Des_Data_Array(k,desTime_adr)-...
                                Des_Data_Array(k,desTime_adr-1))/180.*pi/...
                                (Des_Time_Array(1,desTime_adr)-Des_Time_Array(1,desTime_adr-1));
                        else
                            TargetVar_Struct.U_des(1,j) = 0.;
                        end
                        break;
                    end
                end
            end
        end

        % Parameters to be passed in to the forward function
        params.model  = osimModel;
        params.state  = osimState;
        params.CentSt = TargetVar_Struct;
        params.Control = Control_Func;
%         params.JointReact = aJointReaction;

        disp(['Cycle [' int2str(i) ']/[' int2str(System_Num_step) '] : Calculating from ' num2str(time_start) ' to ' num2str(time_end)]);

        [UpdatedStateVals, UpdatedExtForces] = ForwardOsimFunction_171028(time_start,time_end,params,WholeVarStruct);


        %# Set the initial states of the model
        editableCoordSet = osimModel.updCoordinateSet();

        % Arrange the initial guess by nodes and states
        for j = 1:size(WholeVarStruct.StateVals,1) 
            WholeVarStruct.StateVals(j) = UpdatedStateVals(j);
        end
        
        for j = 1:6
            WholeVarStruct.ContactFr(j) = UpdatedExtForces.Force(j);
        end

        % Use last state variables for next step initial states
        AA = stateStorage.getColumnLabels;
        AB = stateStorage.getLastStateVector().getData;
        disp(['state val [' num2str(AB.get(0)/pi*180.) '], [' num2str(AB.get(2)/pi*180.) ']  ' ]);
        disp('');

        clear AA AB;

    end


    
end




% stop the timer
runtime = toc;
disp(['The runtime was ' num2str(runtime)]);



% Write out storage files containing the states and excitations
if isdir (TargetVar_Struct.outputdir) == 0
    mkdir (TargetVar_Struct.outputdir);
end
outputpath = ['.\' char(TargetVar_Struct.outputdir) '\'];  % use current directory
bufferstamp = char(datetime,'yy_MM_dd-HH_mm');
osimModel.printControlStorage([Sys_Name,'_',bufferstamp,'_excitations.sto']);

% Also write out the results of the analyses to storage files
% aActuation.printResults(['SimOut_',bufferstamp],outputpath,0.01,'.sto');
% aBodyKinematics.printResults(['SimOut_',bufferstamp],outputpath,0.01,'.sto');
% aForceReporter.printResults(['SimOut_',bufferstamp],outputpath,0.01,'.sto');
% aJointReaction.printResults(['SimOut_',bufferstamp],outputpath,0.01,'.sto');
% aKinematics.printResults(['SimOut_',bufferstamp],outputpath,0.01,'.sto');
aStatesReporter.printResults([Sys_Name,'_',bufferstamp],outputpath,0.01,'.sto');
aStatesReporter.printResults([Sys_Name,'_',bufferstamp],'.\.',-1,'.sto');



% Reset the analyses so everything starts fresh for the next
% call to this function
% aActuation.getForceStorage.reset(0);
% aActuation.getPowerStorage.reset(0);
% aActuation.getSpeedStorage.reset(0);
% aBodyKinematics.getPositionStorage.reset(0);
% aBodyKinematics.getVelocityStorage.reset(0);
% aBodyKinematics.getAccelerationStorage.reset(0);
% aForceReporter.getForceStorage.reset(0);
% aKinematics.getAccelerationStorage.reset(0);
% aKinematics.getPositionStorage.reset(0);
% aKinematics.getVelocityStorage.reset(0);
aStatesReporter.getStatesStorage.reset(0);
