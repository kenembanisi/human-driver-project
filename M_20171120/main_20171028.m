
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

if ~file_input
    disp(['Model was not selected']);
    return;
end
                     
[pathstr,Sys_Name,ext] = fileparts(strcat(pathname,file_input));

TargetVar_Struct = InitTargetStruct (pathname,Sys_Name);

osimModel = TargetVar_Struct.Model;

% Add Analyses to the model: ForceReporter and BodyKinematics 
% aActuation = Actuation(osimModel);
% aBodyKinematics = BodyKinematics(osimModel);
% aForceReporter = ForceReporter(osimModel);
% aJointReaction = JointReaction(osimModel);
% aKinematics = Kinematics(osimModel);
% aStatesReporter = StatesReporter(osimModel);
% osimModel.addAnalysis(aActuation);
% osimModel.addAnalysis(aBodyKinematics);
% osimModel.addAnalysis(aForceReporter);
% osimModel.addAnalysis(aJointReaction);
% osimModel.addAnalysis(aKinematics);
% osimModel.addAnalysis(aStatesReporter);


% Check to see if model state is initialized by checking size
if(osimModel.getWorkingState().getNY() == 0)
   osimState = osimModel.initSystem();
else
   osimState = osimModel.updWorkingState(); 
end

% Set the initial states of the model
editableCoordSet = osimModel.updCoordinateSet();

% Arrange the initial guess by nodes and states
for i=1:TargetVar_Struct.Number
    editableCoordSet.get(TargetVar_Struct.Cent_cor_name{1,i}).setValue(osimState, TargetVar_Struct.Q_ini(1,i));
    editableCoordSet.get(TargetVar_Struct.Cent_cor_name{1,i}).setSpeedValue(osimState, TargetVar_Struct.U_ini(1,i));
end


% StateVars of whole model in osimModel definition
WholeVarStruct = InitWholeVarStruct (osimModel, osimState);

Reporter_Struct = InitReportStruct (TargetVar_Struct);

% -- updated every 50ms and can be obtained from the vehicle model --
pedalFile = [Sys_Name,'_pedal.mot'];

% % Go to the folder in the subject's folder where .trc files are
% trc_data_folder = uigetdir(subjectDir, 'Select the folder that contains the marker data files in .trc format.');

pedal_data_folder = 'PedalData';

fullpath = ([pedal_data_folder '\' pedalFile]);

PedalData = getPedalData (fullpath);
% -------------------------------------------------------------------------
% fileoutpath = ['Monitor_Model.osim'];
% osimModel.print(fileoutpath);

% Prepare temporary export model file for debug
% fileoutpath = [Sys_Name,'_Monitor_01.osim'];

% stateStorage = aStatesReporter.getStatesStorage();

UpdatedStateVals = zeros(size(WholeVarStruct.StateLabl,1),1);

UpdatedExtForces = struct();
UpdatedExtForces.Target = '';
UpdatedExtForces.Force = zeros(1,6);

% State_Event = 'Move_to_Brake';

Control_Func      = struct();
Control_Func.File = [Sys_Name,'-',TargetVar_Struct.bufferstamp,'_control.sto'];

State_Storag      = struct();
State_Storag.File33 = [Sys_Name,'-',TargetVar_Struct.bufferstamp,'_state-v33.sto'];
State_Storag.File40 = [Sys_Name,'-',TargetVar_Struct.bufferstamp,'_state-v40.sto'];

params = struct();

% start a timer
tic;


for ii = 2:size(PedalData,2)
   % osimState.setTime(Target_Time.get(ii-2));
    % taking the pedal data and setting for each time step (50ms)
    System_StartTim  = round(osimState.getTime,4);
    System_End_Time  = round(PedalData(1,ii),4);
    
    WholeVarStruct.Time = System_StartTim;
    
    TargetVar_Struct.pedal = PedalData(2,ii);
    TargetVar_Struct.Target_current = getTargetPedals(TargetVar_Struct.pedal);
    
    % check the contact force definition in the last iteration and if it
    % contact body remains same as the desired, keep, else, make zero.
    if ~strcmp(WholeVarStruct.ContactBd,TargetVar_Struct.Target_current)
        WholeVarStruct.ContactFr = zeros(1,6);
    end
    
    Force_Control = mlhdlc_fsm_mealy(sum(WholeVarStruct.ContactFr)); % change to norm

% -----------------------------------------------------------------------
    TargetVar_Struct.Model = osimModel;
    TargetVar_Struct.State = osimState;

    Marker_Param = struct();
    Marker_Param.TargetVar = TargetVar_Struct;
    Marker_Param.start = System_StartTim;
    Marker_Param.end   = System_End_Time;
    Marker_Param.freq  = Trajectory_Frequency; 
    Marker_Param.cycle = round(1/Trajectory_Frequency,5);
    Marker_Param.dir   = 'MarkerData';
    Marker_Param.file  = sprintf('%s_%03d.trc', Sys_Name, ii-1);
    Marker_Param.ContForce = sum(WholeVarStruct.ContactFr); % change to norm
    Marker_Param.Pot_Switch = PotentialField_Switch; 

    Des_Trj_TaskSpace = CreateMarker_RLeg(Marker_Param); % function that computes the desired trajectory as a function of time
    
    [Des_Time,Des_Trj_JntSpace,Des_Trj_JntLabl] = RunIK_RLeg(Marker_Param,WholeVarStruct);
    
    Des_Size = size(Des_Time,2);

    
    System_timestep = round(1./System_Frequency,5);

    System_Num_step = round((System_End_Time-System_StartTim)*System_Frequency);

    disp (['[System]: Time = ' num2str(System_StartTim) '[s] -> ' num2str(System_End_Time) '[s].']);

    State_Storag.time = zeros(1, System_Num_step+1);
    State_Storag.data = zeros(length(WholeVarStruct.StateLabl), System_Num_step+1);
    State_Storag.name_v33 = cell(length(WholeVarStruct.StateLabl),1);
    State_Storag.name_v40 = cell(length(WholeVarStruct.StateLabl),1);
    for i = 1:length(WholeVarStruct.StateLabl)
        State_Storag.name_v40{i} = WholeVarStruct.StateLabl{i};
        SpString = split(WholeVarStruct.StateLabl{i},'/');
        CoordName = char(SpString(2));
        ValueName = char(SpString(3));
        if strcmp(ValueName,'speed')
            State_Storag.name_v33{i} = [CoordName,'_u'];
        else
            State_Storag.name_v33{i} = CoordName;
        end

        State_Storag.data(i,1) = WholeVarStruct.StateVals(i);
    end
    State_Storag.time(1) = System_StartTim;
    for i=1:System_Num_step % setting the time and the control values (addition of a piecewise format to prevent numerical errors)
        State_Storag.time(i+1) = round(System_StartTim + System_timestep * (i), 5);
    end
    
    Control_Func.time = zeros(1, System_Num_step*2+1);
    Control_Func.data = zeros(TargetVar_Struct.Number, System_Num_step*2+1);
    Control_Func.name = cell(TargetVar_Struct.Number,1);
    for i = 1:TargetVar_Struct.Number
        Control_Func.name{i,1} = TargetVar_Struct.Actuator_name{1,i};
        Control_Func.data(i,1) = TargetVar_Struct.Actuator_Data(i);
    end
    Control_Func.time(1) = System_StartTim;
    for i=1:System_Num_step % setting the time and the control values (addition of a piecewise format to prevent numerical errors)
        Control_Func.time(i*2)   = round(System_StartTim + System_timestep * (i-1) + System_timestep*0.1, 5);
        Control_Func.time(i*2+1) = round(System_StartTim + System_timestep * (i), 5);
    end
    
    Reporter_Struct.contact_report.time = zeros(1, System_Num_step+1);
    Reporter_Struct.contact_report.data = zeros(length(Reporter_Struct.contact_report.contact)*8, System_Num_step+1);
    Reporter_Struct.contact_report.time(1) = System_StartTim;
    for i=1:System_Num_step % setting the time and the control values (addition of a piecewise format to prevent numerical errors)
        Reporter_Struct.contact_report.time(i+1) = round(System_StartTim + System_timestep * (i), 5);
    end
    
    
%     Des_Labels = Des_StoreData.getColumnLabels;
    WriteInitials(Marker_Param,WholeVarStruct,State_Storag,Control_Func);


    for i = 1:System_Num_step
        time_start = round(System_StartTim + System_timestep * (i-1), 4);
        time_end   = round(System_StartTim + System_timestep * (i), 4);

        TargetVar_Struct.P_des = getCurr_DesPoint_TaskSpace(Marker_Param,time_end,Des_Trj_TaskSpace);
        
        [TargetVar_Struct.Q_des, TargetVar_Struct.U_des]...
            = getCurr_DesPoint_JointSpace (Marker_Param,time_end,TargetVar_Struct,WholeVarStruct,Des_Trj_JntSpace,Des_Trj_JntLabl);
            
        
        % Parameters to be passed in to the forward function
        params.model  = osimModel;
        params.state  = osimState;
        params.TargetVar_Struct = TargetVar_Struct;
        params.Control = Control_Func;
        params.Reporter_Struct = Reporter_Struct;
%         params.JointReact = aJointReaction;

        [qq,rs] = quorem(sym(i),sym(10));
        rr = double(rs);
        if rr == 0
            disp(['Cycle [' int2str(i) ']/[' int2str(System_Num_step) '] : Calculating from ' num2str(time_start) ' to ' num2str(time_end)]);
        end
        
        Force_Control = mlhdlc_fsm_mealy(sum(WholeVarStruct.ContactFr));

        Current_Sub_Point = osimModel.getBodySet().get(TargetVar_Struct.TargetSub_Pointer{getTargetCurID(TargetVar_Struct),1}).getPositionInGround(osimState);
        Pointer_Target_1 = osimModel.getBodySet().get(TargetVar_Struct.TargetDes_Pointer{getTargetCurID(TargetVar_Struct),1}).getPositionInGround(osimState);
        
        if Force_Control
            Comp_KPs = zeros(1,3);
            Comp_KPs(1) = 25000; Comp_KPs(2) = 25000; Comp_KPs(3) = 1000; comp_KV = 5;
            [UpdatedStateVals, UpdatedForces, UpdatedControls] = Forward_InverseCont_171028(time_start,time_end,params,WholeVarStruct,Comp_KPs,comp_KV);
        elseif CheckDistVec3(Current_Sub_Point,Pointer_Target_1,0.02)
            [UpdatedStateVals, UpdatedForces, UpdatedControls] = ForwardOsimFunction_171028(time_start,time_end,params,WholeVarStruct);
        else
            Comp_KPs = zeros(1,3);
            Comp_KPs(1) = 5000; Comp_KPs(2) = 5000; Comp_KPs(3) = 1000; comp_KV = 2;
            [UpdatedStateVals, UpdatedForces, UpdatedControls] = Forward_InverseCont_171028(time_start,time_end,params,WholeVarStruct,Comp_KPs,comp_KV);
        end

        if osimState.getTime ~= time_end
            osimState.setTime(time_end);
        end

        % Arrange the initial guess by nodes and states
        WholeVarStruct.StateVals = UpdatedStateVals;
        
        Reporter_Struct.contact_report.data(:,i+1) = UpdatedForces;
        
        OutForces = zeros(1,6);
        ForceSet = osimModel.getForceSet();
        Target_address = getTargetCurID(TargetVar_Struct);
   
        for iii = 1:TargetVar_Struct.ContactNumber
            counter = 1;
            ContactSet = ForceSet.get(TargetVar_Struct.TargetContact{Target_address,iii});
            for jj = 1:length(Reporter_Struct.contact_report.dataLabls)
                split_str = split(Reporter_Struct.contact_report.dataLabls{jj,1},'.');
                if ( strcmp(split_str(1),TargetVar_Struct.TargetContact{Target_address,iii})...
                        && strcmp(split_str(2),TargetVar_Struct.TargetBody{Target_address,1}) )
                    OutForces(counter) = OutForces(counter) + Reporter_Struct.contact_report.data(jj,i+1);
                    counter = counter + 1;
                    if counter>6
                        break;
                    end
                end
            end
        end
        UpdatedExtForces.Target = TargetVar_Struct.TargetBody{Target_address,1};
        UpdatedExtForces.Force = OutForces;

        
        WholeVarStruct.ContactBd = UpdatedExtForces.Target;
        WholeVarStruct.ContactFr = UpdatedExtForces.Force;

        State_Storag.data(:,i+1) = WholeVarStruct.StateVals(:,1);
        Control_Func.data = UpdatedControls;

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

    end
    
    WriteStates(State_Storag);
    WriteReportCont(Reporter_Struct);
    [TargetVar_Struct.Actuator_Time,TargetVar_Struct.Actuator_Data] = WriteControl(Control_Func);
    
end




% stop the timer
runtime = toc;
disp(['The runtime was ' num2str(runtime)]);



% Write out storage files containing the states and excitations
if isdir (TargetVar_Struct.outputdir) == 0
    mkdir (TargetVar_Struct.outputdir);
end
outputpath = ['.\' char(TargetVar_Struct.outputdir) '\'];  % use current directory
osimModel.printControlStorage([Sys_Name,'_',TargetVar_Struct.bufferstamp,'_excitations.sto']);

% Also write out the results of the analyses to storage files
% aActuation.printResults(['SimOut_',bufferstamp],outputpath,0.01,'.sto');
% aBodyKinematics.printResults(['SimOut_',bufferstamp],outputpath,0.01,'.sto');
% aForceReporter.printResults(['SimOut_',bufferstamp],outputpath,0.01,'.sto');
% aJointReaction.printResults(['SimOut_',bufferstamp],outputpath,0.01,'.sto');
% aKinematics.printResults(['SimOut_',bufferstamp],outputpath,0.01,'.sto');
% aStatesReporter.printResults([Sys_Name,'_',bufferstamp],outputpath,0.01,'.sto');
% aStatesReporter.printResults([Sys_Name,'_',bufferstamp],'.\.',-1,'.sto');



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
% aStatesReporter.getStatesStorage.reset(0);
