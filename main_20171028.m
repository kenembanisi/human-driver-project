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
System_End_Time  = 000.50;
System_Frequency = fix(1000.);

kp = 100;
kv = 20;

if System_Frequency < 1
    System_Frequency = 1.;
end


System_timestep = 1./System_Frequency;

System_Num_step = fix((System_End_Time-System_StartTim)*System_Frequency);


% Import the OpenSim modeling classes
import org.opensim.modeling.*

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

CentStruct.Cent_cor_name(1,1) = cell({'joint_1'});
CentStruct.Cent_cor_name(1,2) = cell({'joint_2'});
CentStruct.Cent_act_name(1,1) = cell({'joint_1_actuator'});
CentStruct.Cent_act_name(1,2) = cell({'joint_2_actuator'});

CentStruct.Q_des(1,1) =  80. * pi / 180.;
CentStruct.U_des(1,1) =  0.0 * pi / 180.;
CentStruct.Q_des(1,2) = -80. * pi / 180.;
CentStruct.U_des(1,2) =  0.0 * pi / 180.;

Control_Func.time = zeros(1, System_Num_step*2+1);
Control_Func.data = zeros(Num_Cent_Cnt, System_Num_step*2+1);
Control_Func.name = CentStruct.Cent_act_name;
Control_Func.time(1) = System_StartTim;
for i=1:System_Num_step
    Control_Func.time(i*2)   = System_StartTim + System_timestep * (i-1) + System_timestep*0.01;
    Control_Func.time(i*2+1) = System_StartTim + System_timestep * (i);
end




for i=1:Num_Cent_Cnt
    Coord = osimModel.getCoordinateSet().get(CentStruct.Cent_cor_name(1,i));
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
    Coord = osimModel.getCoordinateSet().get(CentStruct.Cent_cor_name(1,i));
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
    Coord = osimModel.getCoordinateSet().get(CentStruct.Cent_cor_name(1,i));
    Coord.setValue ( osimState, CentStruct.Q_ini(1,i) );
    Coord.setSpeedValue ( osimState, CentStruct.U_ini(1,i) );
end

%# Set the initial states of the model
editableCoordSet = osimModel.updCoordinateSet();

% Arrange the initial guess by nodes and states
for i=1:Num_Cent_Cnt
    editableCoordSet.get(CentStruct.Cent_cor_name(1,i)).setValue(osimState, CentStruct.Q_ini(1,i));
    editableCoordSet.get(CentStruct.Cent_cor_name(1,i)).setSpeedValue(osimState, CentStruct.U_ini(1,i));
end


JointSet = osimModel.getJointSet();
CoordSet = osimModel.getCoordinateSet();
StateValNames      = osimModel.getStateVariableNames();

InitStruct = struct();
InitStruct.JointSet  = cell(JointSet.getSize,1);
InitStruct.CoordSet  = cell(CoordSet.getSize,1);
InitStruct.StateLabl = cell(StateValNames.getSize,1);
InitStruct.StateVals = zeros(StateValNames.getSize,1);

for i = 1:StateValNames.getSize
    InitStruct.StateLabl(i,1) = cell(StateValNames.getitem(i-1));
    SpString = split(InitStruct.StateLabl(i,1),'/');
    JointName = char(SpString(1));
    CoordName = char(SpString(2));
    ValueName = char(SpString(3));
    Coord = osimModel.getCoordinateSet().get(CoordName);
    if strcmp(ValueName, 'value')
        InitStruct.StateVals(i,1) = Coord.getValue(osimState);        
    else
        InitStruct.StateVals(i,1) = Coord.getSpeedValue(osimState);        
    end
end

for i = 1:CoordSet.getSize
    InitStruct.CoordSet(i,1) = cell(CoordSet.get(i-1));
end

for i = 1:JointSet.getSize
    InitStruct.JointSet(i,1) = cell(JointSet.get(i-1));
end





% Parameters to be passed in to the forward function
params.model  = osimModel;
params.state  = osimState;
params.CentSt = CentStruct;
params.Control = Control_Func;
params.JointReact = aJointReaction;

stateStorage = aStatesReporter.getStatesStorage();

% start a timer
tic;

for i = 1:System_Num_step
    time_start = System_StartTim + System_timestep * (i-1);
    time_end = System_StartTim + System_timestep * (i);

    disp(['Cycle [' int2str(i) ']/[' int2str(System_Num_step) '] : Calculating from ' num2str(time_start) ' to ' num2str(time_end)]);

    ForwardOsimFunction_171028(time_start,time_end,params,InitStruct);


    % Use last state variables for next step initial states
    AA = stateStorage.getColumnLabels;
    AB = stateStorage.getLastStateVector().getData;
    for k=1:AB.getSize
        for j = 1:size(InitStruct.StateLabl,1)
           if (strcmp(InitStruct.StateLabl(j),AA.getitem(k)))
               InitStruct.StateVals(j) = AB.getitem(k-1);
           end
        end
    end
    disp(['state val [' num2str(AB.get(0)/pi*180.) '], [' num2str(AB.get(2)/pi*180.) ']  ' ]);
    disp('');

    clear AA AB;
    
end



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
