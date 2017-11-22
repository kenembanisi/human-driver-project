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

format long;

% Node and timing information
System_StartTim  = 0.00;
System_End_Time  = 000.90;
Trajectory_Frequency = round(100.);
System_Frequency = round(1000.);

PotentialField_Switch = true;

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
UpdatedExtForces = struct();
UpdatedExtForces.Target = '';
UpdatedExtForces.Force = zeros(1,6);

State_Event = 'Move_to_Brake';

% start a timer
tic;


for ii = 2:size(PedalData,2)
   % osimState.setTime(Target_Time.get(ii-2));
    
    System_StartTim  = round(osimState.getTime,4);
    System_End_Time  = round(PedalData(1,ii),4);
    
    TargetVar_Struct.pedal = PedalData(2,ii);
    if TargetVar_Struct.pedal >= 0
        TargetVar_Struct.Target_current = 'G_Pedal';
    else
        TargetVar_Struct.Target_current = 'B_Pedal';
    end

    if ~strcmp(WholeVarStruct.ContactBd,TargetVar_Struct.Target_current)
        WholeVarStruct.ContactFr = zeros(1,6);
    end
    
    Force_Control = mlhdlc_fsm_mealy(sum(WholeVarStruct.ContactFr));
    
    Marker_Param = struct();
    Marker_Param.model = osimModel;
    Marker_Param.state = osimState;
    Marker_Param.frame = Frame;
    Marker_Param.start = System_StartTim;
    Marker_Param.end   = System_End_Time;
    Marker_Param.freq  = Trajectory_Frequency; 
    Marker_Param.cycle = round(1/Trajectory_Frequency,5);
    Marker_Param.dir   = 'MarkerData';
    Marker_Param.Sysname = Sys_Name;
    Marker_Param.file  = sprintf('%s_%03d.trc', Sys_Name, ii-1);
    Marker_Param.TargetVar = TargetVar_Struct;
    Marker_Param.ContForce = sum(WholeVarStruct.ContactFr);
    Marker_Param.Pot_Switch = PotentialField_Switch; 

    [Des_Trj_TaskSpace,Next] = CreateMarker_RLeg(Marker_Param);
    Frame = Next;
    
    IK_Param = struct();
    IK_Param.model     = osimModel;
    IK_Param.read_dir  = Marker_Param.dir;
    IK_Param.read_file = Marker_Param.file;
    IK_Param.setup_file= [Sys_Name,'_IK_Setup_Master.xml'];
    IK_Param.WholeVarStruct = WholeVarStruct;

    
    [Des_Time,Des_Trj_JntSpace,Des_Trj_JntLabl] = RunIK_RLeg(IK_Param,Marker_Param);
    
    Des_Size = size(Des_Time,2);

    
    System_timestep = round(1./System_Frequency,5);

    System_Num_step = round((System_End_Time-System_StartTim)*System_Frequency);

    disp (['[System]: Time = ' num2str(System_StartTim) '[s] -> ' num2str(System_End_Time) '[s].']);

    
    Control_Func.time = zeros(1, System_Num_step*2+1);
    Control_Func.data = zeros(TargetVar_Struct.Number, System_Num_step*2+1);
    Control_Func.name = TargetVar_Struct.Cent_act_name;
    Control_Func.time(1) = System_StartTim;
    for i=1:System_Num_step
        Control_Func.time(i*2)   = System_StartTim + System_timestep * (i-1) + System_timestep*0.1;
        Control_Func.time(i*2+1) = System_StartTim + System_timestep * (i);
    end
    
    
%     Des_Labels = Des_StoreData.getColumnLabels;


    for i = 1:System_Num_step
        time_start = System_StartTim + System_timestep * (i-1);
        time_end = System_StartTim + System_timestep * (i);

        TargetVar_Struct.P_des = getCurr_DesPoint_TaskSpace(Marker_Param,time_end,Des_Trj_TaskSpace);
        
        [TargetVar_Struct.Q_des, TargetVar_Struct.U_des]...
            = getCurr_DesPoint_JointSpace (Marker_Param,time_end,TargetVar_Struct,WholeVarStruct,Des_Trj_JntSpace,Des_Trj_JntLabl);
            
        
        % Parameters to be passed in to the forward function
        params.model  = osimModel;
        params.state  = osimState;
        params.CentSt = TargetVar_Struct;
        params.Control = Control_Func;
%         params.JointReact = aJointReaction;

        [qq,rs] = quorem(sym(i),sym(10));
        rr = double(rs);
        if rr == 0
            disp(['Cycle [' int2str(i) ']/[' int2str(System_Num_step) '] : Calculating from ' num2str(time_start) ' to ' num2str(time_end)]);
        end

        [UpdatedStateVals, UpdatedExtForces] = ForwardOsimFunction_171028(time_start,time_end,params,WholeVarStruct);


        % Arrange the initial guess by nodes and states
        WholeVarStruct.StateVals = UpdatedStateVals;
        
        WholeVarStruct.ContactBd = UpdatedExtForces.Target;
        WholeVarStruct.ContactFr = UpdatedExtForces.Force;

        % Use last state variables for next step initial states
        if rr == 0
            disp (['Target pedal position = ' num2str(TargetVar_Struct.pedal) '[%]']);
            disp ([char(WholeVarStruct.StateLabl{1}) ' angle = ' num2str(WholeVarStruct.StateVals(1)/pi*180.) '[deg]' ]);
            disp ([char(WholeVarStruct.StateLabl{3}) ' angle = ' num2str(WholeVarStruct.StateVals(3)/pi*180.) '[deg]' ]);
            disp('');
            
            disp_string = sprintf('contact forces on %s = [ ',WholeVarStruct.ContactBd);
            for j = 1:6
                disp_string = char([disp_string num2str(WholeVarStruct.ContactFr(j)) ', ']);
            end
            disp_string = char([disp_string ' ]']);
            disp(disp_string);
            disp(' ');
        end

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
