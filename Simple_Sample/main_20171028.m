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

kp = 100;
kv = 20;

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

% Create an OpenSim model from an OSIM file
Model_In = ['TwoLinkArmModel.osim'];
osimModel = Model(Model_In);

fileoutpath = ['Monitor_Model.osim'];
osimModel.print(fileoutpath);

output_Dir = 'Result';

% Prepare temporary export model file for debug
fileoutpath = [Model_In(1:end-5),'_Monitor_01.osim'];


% Add Analyses to the model: ForceReporter and BodyKinematics 
aActuation = Actuation(osimModel);
aBodyKinematics = BodyKinematics(osimModel);
aForceReporter = ForceReporter(osimModel);
aJointReaction = JointReaction(osimModel);
aKinematics = Kinematics(osimModel);
aStatesReporter = StatesReporter(osimModel);
osimModel.addAnalysis(aActuation);
osimModel.addAnalysis(aBodyKinematics);
osimModel.addAnalysis(aForceReporter);
osimModel.addAnalysis(aJointReaction);
osimModel.addAnalysis(aKinematics);
osimModel.addAnalysis(aStatesReporter);


% Initialize the model (build the system and intialize the state)
osimState = osimModel.initSystem();

% Get the number of states from the model;
% in this case the number of controls equals the number of muscles

Num_Cent_Cnt = 2;
CentStruct = struct();
CentStruct.Number = Num_Cent_Cnt;
CentStruct.Cent_val_name = cell(1, Num_Cent_Cnt);
CentStruct.Cent_cor_name = cell(1, Num_Cent_Cnt);
CentStruct.Cent_act_name = cell(1, Num_Cent_Cnt);
CentStruct.ValAd = zeros(1, Num_Cent_Cnt);
CentStruct.Q_des = zeros(1, Num_Cent_Cnt);
CentStruct.U_des = zeros(1, Num_Cent_Cnt);
CentStruct.Q_ini = zeros(1, Num_Cent_Cnt);
CentStruct.U_ini = zeros(1, Num_Cent_Cnt);
CentStruct.ContV = zeros(1, Num_Cent_Cnt);
CentStruct.kp = kp * ones(1, Num_Cent_Cnt);
CentStruct.kv = kv * ones(1, Num_Cent_Cnt);

CentStruct.Cent_cor_name{1,1} = 'joint_1';
CentStruct.Cent_cor_name{1,2} = 'joint_2';
CentStruct.Cent_act_name{1,1} = 'joint_1_actuator';
CentStruct.Cent_act_name{1,2} = 'joint_2_actuator';

CentStruct.Q_des(1,1) =  80. * pi / 180.;
CentStruct.U_des(1,1) =  0.0 * pi / 180.;
CentStruct.Q_des(1,2) = -80. * pi / 180.;
CentStruct.U_des(1,2) =  0.0 * pi / 180.;




for i=1:Num_Cent_Cnt
    Coord = osimModel.getCoordinateSet().get(CentStruct.Cent_cor_name{1,i});
    CentStruct.Q_ini(1,i) = Coord.getValue(osimState);
    CentStruct.U_ini(1,i) = Coord.getSpeedValue(osimState);
    Coord.setValue(osimState, 0.0);
    Coord.setSpeedValue(osimState, 0.0);
end

% If I need to change initial joint angles, input values here.
CentStruct.Q_ini(1,1) = 90. * pi / 180.;
CentStruct.Q_ini(1,2) = 45. * pi / 180.;



% Solve difference of variable orders between osimModel and osimState.
osimModel.realizeVelocity(osimState);
for i=1:Num_Cent_Cnt
    Coord = osimModel.getCoordinateSet().get(CentStruct.Cent_cor_name{1,i});
    Coord.setValue(osimState, 1.);
    Coord.setSpeedValue(osimState, -8888.8);
    osimModel.realizeVelocity(osimState);
    tempQ = osimState.getQ;
    tempU = osimState.getU;
    for j=1:size(tempQ)
        if tempU.get(j-1) == -8888.8
            CentStruct.ValAd(1,i) = j;
            break;
        end
    end
    Coord.setValue(osimState, 0.0);
    Coord.setSpeedValue(osimState, 0.0);
    osimState.getQ
end

% Return initial state variables into osimModel.
for i=1:Num_Cent_Cnt
    Coord = osimModel.getCoordinateSet().get(CentStruct.Cent_cor_name{1,i});
    Coord.setValue ( osimState, CentStruct.Q_ini(1,i) );
    Coord.setSpeedValue ( osimState, CentStruct.U_ini(1,i) );
end


%# Set the initial states of the model
editableCoordSet = osimModel.updCoordinateSet();

% Arrange the initial guess by nodes and states
for i=1:Num_Cent_Cnt
    editableCoordSet.get(CentStruct.Cent_cor_name{1,i}).setValue(osimState, CentStruct.Q_ini(1,i));
    editableCoordSet.get(CentStruct.Cent_cor_name{1,i}).setSpeedValue(osimState, CentStruct.U_ini(1,i));
end


JointSet = osimModel.getJointSet();
CoordSet = osimModel.getCoordinateSet();
StateValNames      = osimModel.getStateVariableNames();

InitStruct = struct();
InitStruct.JointSet  = cell(JointSet.getSize,1);
InitStruct.CoordSet  = cell(CoordSet.getSize,1);
InitStruct.StateLabl = cell(StateValNames.getSize,1);
InitStruct.StateVals = zeros(StateValNames.getSize,1);
UpdatedStateVals = zeros(StateValNames.getSize,1);

for i = 1:StateValNames.getSize
    InitStruct.StateLabl{i,1} = StateValNames.getitem(i-1);
    SpString = split(InitStruct.StateLabl{i,1},'/');
    JointName = SpString(1);
    CoordName = SpString(2);
    ValueName = SpString(3);
    Coord = osimModel.getCoordinateSet().get(CoordName);
    if strcmp(ValueName, 'value')
        InitStruct.StateVals(i,1) = Coord.getValue(osimState);        
    else
        InitStruct.StateVals(i,1) = Coord.getSpeedValue(osimState);        
    end
end

for i = 1:CoordSet.getSize
    InitStruct.CoordSet{i,1} = CoordSet.get(i-1);
end

for i = 1:JointSet.getSize
    InitStruct.JointSet{i,1} = JointSet.get(i-1);
end


pedalFile = 'pedal_input_master.mot';

% % Go to the folder in the subject's folder where .trc files are
% trc_data_folder = uigetdir(subjectDir, 'Select the folder that contains the marker data files in .trc format.');

pedal_data_folder = 'PedalData';

fullpath = ([pedal_data_folder '\' pedalFile]);

% Get trc data to determine time range
PedalData = Storage(fullpath);

Target_Time = ArrayDouble();
Pedal_Position  = ArrayDouble();
Num_Target = PedalData.getTimeColumn(Target_Time);
Pedal_Name = PedalData.getColumnLabels().get(1);
PedalData.getDataColumn(Pedal_Name,Pedal_Position);

Frame = 1;

stateStorage = aStatesReporter.getStatesStorage();

% start a timer
tic;


for ii = 2:Num_Target
   % osimState.setTime(Target_Time.get(ii-2));
    
    System_StartTim  = osimState.getTime;
    System_End_Time  = Target_Time.get(ii-1);

    Marker_Param.model = osimModel;
    Marker_Param.state = osimState;
    Marker_Param.frame = Frame;
    Marker_Param.pedal = Pedal_Position.get(ii-1);
    Marker_Param.start = System_StartTim;
    Marker_Param.end   = System_End_Time;
    Marker_Param.dir   = 'MarkerData';
    Marker_Param.file  = sprintf('two_link_pedal_%03d.trc', ii-1);
    
    Next = CreateMarker_TwoLink(Marker_Param);
    Frame = Next;
    
    IK_Param.model     = osimModel;
    IK_Param.read_dir  = Marker_Param.dir;
    IK_Param.read_file = Marker_Param.file;
    IK_Param.out_dir   = 'Out_IK';
   
    
    IK_outfile = RunIK_TwoLink(IK_Param);

    motfilepath = IK_outfile;
    Des_StoreData = Storage(motfilepath);
    
    Des_Time = ArrayDouble();
    Des_Size = Des_StoreData.getTimeColumn (Des_Time);

    Des_Time_Vector = Des_Time.getAsVector;

    Des_Time_Array = osimVectorToArray(Des_Time_Vector);
    Des_Data_Array = zeros(size(InitStruct.CoordSet,1),Des_Size);
    Des_Labl_Array = cell(size(InitStruct.CoordSet,1),1);

    for i = 1:size(InitStruct.CoordSet,1)
        coordvalue = ArrayDouble();
        Des_StoreData.getDataColumn(char(InitStruct.CoordSet{i,1}),coordvalue);
        coordvect = coordvalue.getAsVector;
        Des_Data_Array(i,:) = osimVectorToArray(coordvect);
        Des_Labl_Array{i,1} = char(InitStruct.CoordSet{i,1});
    end
    
    
    System_timestep = 1./System_Frequency;

    System_Num_step = fix((System_End_Time-System_StartTim)*System_Frequency);


    
    Control_Func.time = zeros(1, System_Num_step*2+1);
    Control_Func.data = zeros(Num_Cent_Cnt, System_Num_step*2+1);
    Control_Func.name = CentStruct.Cent_act_name;
    Control_Func.time(1) = System_StartTim;
    for i=1:System_Num_step
        Control_Func.time(i*2)   = System_StartTim + System_timestep * (i-1) + System_timestep*0.01;
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

            for j = 1:CentStruct.Number
                for k = 1:size(InitStruct.CoordSet,1)
                    if strcmp(CentStruct.Cent_cor_name{1,j},Des_Labl_Array{k,1})
                        CentStruct.Q_des(1,j) = Des_Data_Array(k,desTime_adr)/180*pi;
                        if desTime_adr > 1
                            CentStruct.U_des(1,j) = (Des_Data_Array(k,desTime_adr)-...
                                Des_Data_Array(k,desTime_adr-1))/180.*pi/...
                                (Des_Time_Array(1,desTime_adr)-Des_Time_Array(1,desTime_adr-1));
                        else
                            CentStruct.U_des(1,j) = 0.;
                        end
                        break;
                    end
                end
            end
        end

        % Parameters to be passed in to the forward function
        params.model  = osimModel;
        params.state  = osimState;
        params.CentSt = CentStruct;
        params.Control = Control_Func;
        params.JointReact = aJointReaction;

        disp(['Cycle [' int2str(i) ']/[' int2str(System_Num_step) '] : Calculating from ' num2str(time_start) ' to ' num2str(time_end)]);

        UpdatedStateVals = ForwardOsimFunction_171028(time_start,time_end,params,InitStruct);


        %# Set the initial states of the model
        editableCoordSet = osimModel.updCoordinateSet();

        % Arrange the initial guess by nodes and states
        for j = 1:size(InitStruct.StateVals,1) 
            InitStruct.StateVals(j) = UpdatedStateVals(j);
        end

        % Use last state variables for next step initial states
        AA = stateStorage.getColumnLabels;
        AB = stateStorage.getLastStateVector().getData;
    %     for k=1:AB.getSize
    %         for j = 1:size(InitStruct.StateLabl,1)
    %            if (strcmp(InitStruct.StateLabl{j,1},AA.getitem(k)))
    %                InitStruct.StateVals(j) = AB.getitem(k-1);
    %            end
    %         end
    %     end
        disp(['state val [' num2str(AB.get(0)/pi*180.) '], [' num2str(AB.get(2)/pi*180.) ']  ' ]);
        disp('');

        clear AA AB;

    end


    
end

% motfilepath='Out_IK\two_link_04_ik.mot';
% Des_StoreData = Storage(motfilepath);
% 
% Des_Time = ArrayDouble();
% Des_Size = Des_StoreData.getTimeColumn (Des_Time);
% 
% Des_Time_Vector = Des_Time.getAsVector;
% 
% Des_Time_Array = osimVectorToArray(Des_Time_Vector);
% Des_Data_Array = zeros(size(InitStruct.CoordSet,1),Des_Size);
% Des_Labl_Array = cell(size(InitStruct.CoordSet,1),1);
% 
% for i = 1:size(InitStruct.CoordSet,1)
%     coordvalue = ArrayDouble();
%     Des_StoreData.getDataColumn(char(InitStruct.CoordSet{i,1}),coordvalue);
%     coordvect = coordvalue.getAsVector;
%     Des_Data_Array(i,:) = osimVectorToArray(coordvect);
%     Des_Labl_Array{i,1} = char(InitStruct.CoordSet{i,1});
% end
% 
% Des_Labels = Des_StoreData.getColumnLabels;
% 
% 
% 
% % Parameters to be passed in to the forward function
% params.model  = osimModel;
% params.state  = osimState;
% params.CentSt = CentStruct;
% params.Control = Control_Func;
% params.JointReact = aJointReaction;
% 
% stateStorage = aStatesReporter.getStatesStorage();
% 
% % start a timer
% tic;
% 
% for i = 1:System_Num_step
%     time_start = System_StartTim + System_timestep * (i-1);
%     time_end = System_StartTim + System_timestep * (i);
%     
%     if Des_Size
%         j = 0.;
%         if time_end < Des_Time_Array(1,1)
%             desTime_adr = 1;
%         elseif time_end > Des_Time_Array(1,Des_Size)
%             desTime_adr = Des_Size;
%         else
%             desTime_adr = Des_Size;
%             for j = 1:Des_Size
%                 if time_end < Des_Time_Array(1,j)
%                     desTime_adr = j;
%                     break;
%                 end
%             end
%         end
%         
%         for j = 1:CentStruct.Number
%             for k = 1:size(InitStruct.CoordSet,1)
%                 if strcmp(CentStruct.Cent_cor_name{1,j},Des_Labl_Array{k,1})
%                     CentStruct.Q_des(1,j) = Des_Data_Array(k,desTime_adr)/180*pi;
%                     if desTime_adr > 1
%                         CentStruct.U_des(1,j) = (Des_Data_Array(k,desTime_adr)-...
%                             Des_Data_Array(k,desTime_adr-1))/180.*pi/...
%                             (Des_Time_Array(1,desTime_adr)-Des_Time_Array(1,desTime_adr-1));
%                     else
%                         CentStruct.U_des(1,j) = 0.;
%                     end
%                     break;
%                 end
%             end
%         end
%     end
%     
%     params.CentSt = CentStruct;
% 
%     disp(['Cycle [' int2str(i) ']/[' int2str(System_Num_step) '] : Calculating from ' num2str(time_start) ' to ' num2str(time_end)]);
% 
%     UpdatedStateVals = ForwardOsimFunction_171028(time_start,time_end,params,InitStruct);
% 
% 
%     %# Set the initial states of the model
%     editableCoordSet = osimModel.updCoordinateSet();
% 
%     % Arrange the initial guess by nodes and states
%     for ii = 1:size(InitStruct.StateVals,1) 
%         InitStruct.StateVals(ii) = UpdatedStateVals(ii);
%     end
%     
%     % Use last state variables for next step initial states
%     AA = stateStorage.getColumnLabels;
%     AB = stateStorage.getLastStateVector().getData;
% %     for k=1:AB.getSize
% %         for j = 1:size(InitStruct.StateLabl,1)
% %            if (strcmp(InitStruct.StateLabl{j,1},AA.getitem(k)))
% %                InitStruct.StateVals(j) = AB.getitem(k-1);
% %            end
% %         end
% %     end
%     disp(['state val [' num2str(AB.get(0)/pi*180.) '], [' num2str(AB.get(2)/pi*180.) ']  ' ]);
%     disp('');
% 
%     clear AA AB;
%     
% end
% 


% stop the timer
runtime = toc;
disp(['The runtime was ' num2str(runtime)]);



% Write out storage files containing the states and excitations
if isdir (output_Dir) == 0
    mkdir (output_Dir);
end
outputpath = ['.\' char(output_Dir) '\'];  % use current directory
bufferstamp = char(datetime,'yy_MM_dd-HH_mm');
osimModel.printControlStorage(['SimOut_',bufferstamp,'_excitations.sto']);

% Also write out the results of the analyses to storage files
aActuation.printResults(['SimOut_',bufferstamp],outputpath,0.01,'.sto');
aBodyKinematics.printResults(['SimOut_',bufferstamp],outputpath,0.01,'.sto');
aForceReporter.printResults(['SimOut_',bufferstamp],outputpath,0.01,'.sto');
aJointReaction.printResults(['SimOut_',bufferstamp],outputpath,0.01,'.sto');
aKinematics.printResults(['SimOut_',bufferstamp],outputpath,0.01,'.sto');
aStatesReporter.printResults(['SimOut_',bufferstamp],outputpath,0.01,'.sto');
aStatesReporter.printResults(['SimOut_',bufferstamp],'.\.',-1,'.sto');



% Reset the analyses so everything starts fresh for the next
% call to this function
aActuation.getForceStorage.reset(0);
aActuation.getPowerStorage.reset(0);
aActuation.getSpeedStorage.reset(0);
aBodyKinematics.getPositionStorage.reset(0);
aBodyKinematics.getVelocityStorage.reset(0);
aBodyKinematics.getAccelerationStorage.reset(0);
aForceReporter.getForceStorage.reset(0);
aKinematics.getAccelerationStorage.reset(0);
aKinematics.getPositionStorage.reset(0);
aKinematics.getVelocityStorage.reset(0);
aStatesReporter.getStatesStorage.reset(0);
