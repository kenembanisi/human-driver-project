% ----------------------------------------------------------------------- %
% main_type1_w_prescmot_2.m
%
% Author: Hideyuki Kimpara, Worcester Polytechnic Institute
%
% This code uses the Matlab interface to the OpenSim API to run a trial
% to connect simulink simulator and OpenSim analysis. 
% This example borrows from other Matlab and C++ examples avaiable from
% the OpenSim project written by several people including, but not limited
% to: Brian Umberger, Ajay Seth, Ayman Habib and Sam Hamner.
% ----------------------------------------------------------------------- %

clear

% Node and timing information
Simulink_timestep = 0.02;
TimeData = struct();
TimeData.time_start = 0.0;
% TimeData.time_end   = TimeData.time_start + Simulink_timestep;
TimeData.time_end   = 100.0;


% Import the OpenSim modeling classes
import org.opensim.modeling.*

% Create an OpenSim model from an OSIM file
Model_In = '../Model/Driver_noMus-20170508.osim'
fileoutpath = [Model_In(1:end-5),'_Prescribed-g.osim'];
osimModel = Model(Model_In);


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

% Load the .mot file that contain the prescribed motion
[file_input, pathname] = uigetfile({'*.mot', 'OpenSim motion Files (*.mot)'}, ...
                         'Select the file for prescribed motion','MultiSelect', 'off');
motfilepath = strcat(pathname,file_input); 

if (size(motfilepath,2))
    % motfilepath = 'Power_tx_mot.mot';
    mot_func = importdata(motfilepath); % import motion data (1st column is time)
    old_time_m = mot_func.data(:,1);         % time 
    old_data_m = mot_func.data(:,2:end);     % the 6 states
    old_text_m = mot_func.textdata(7,2:end); % states names


    % get coordinate set from model, and count the number of coordinates
    modelCoordSet = osimModel.getCoordinateSet();
    nCoords = modelCoordSet.getSize();

    % for all coordinates in the model, create a function and prescribe
    for i=0:nCoords-1

        % Get the coordinate set from the model
        currentcoord = modelCoordSet.get(i);

        for j=1:size(old_data_m,2)
            if strcmp(currentcoord.getName(), old_text_m(j))

                % Check if it is a rotational or translational coordinate
                motion = currentcoord.getMotionType();

                % construct a SimmSpline object (previously NaturalCubicSpline)
                Spline = SimmSpline();

                %Now to write Time and coordvalue to Spline
                if strcmp(motion,'Rotational')
                    % if the motion type is rotational we must convert to radians from degrees
                    for j = 1:size(old_data_m,1)
                        Spline.addPoint(old_time_m(j),old_data_m(j)/(180/pi));
                    end
                else % else we assume it's translational and can be left 'as is'
                    for j = 1:size(old_data_m,1)
                        Spline.addPoint(old_time_m(j),old_data_m(j));
                    end
                end

                % Add the SimmSpline to the PrescribedFunction of the Coordinate
                % being edited
                currentcoord.setPrescribedFunction(Spline);
                currentcoord.setDefaultIsPrescribed(1);
            end
        end

    end

end


% Initialize the model (build the system and intialize the state)
osimState = osimModel.initSystem();

% Get the number of states, coordinates, muscles and controls from the model;
% in this case the number of controls equals the number of muscles
Nstates       = osimModel.getNumStateVariables();
Ncontrols     = osimModel.getNumControls();
Ncoord        = osimModel.getNumCoordinates(); 
model_muscles = osimModel.getMuscles();
Nmuscles      = model_muscles.getSize();




states_all = cell(Nstates,1);
for i = 1:Nstates
   states_all(i,1) = cell(osimModel.getStateVariableNames().getitem(i-1));
end


% Load the .sto file that contain the initial guess for the states
[file_input, pathname] = uigetfile({'*.sto', 'OpenSim States Files (*.sto)'}, ...
    'Select the initial states file','MultiSelect', 'off');

if (file_input)
    temp_s = importdata(strcat(pathname,file_input)); % import states data

    old_time_s = temp_s.data(:,1);         % time 
    old_data_s = temp_s.data(:,2:end);     % the 6 states
    old_text_s = temp_s.textdata(7,2:end); % states names

    % Arrange the initial guess by nodes and states
    x0_temp = zeros(Nstates,1);  % pre-allocate space
    chk_counter =0;
    for j = 1:size(states_all,1) 
        for k = 1:size(old_data_s,2)
            if strcmp(old_text_s(k),states_all(j)) == 1,
                % interpolate initial guess to the defined temporal grid
                x0_temp(j) = old_data_s(1,k);
                chk_counter = chk_counter+1;
            end
        end
        if chk_counter ~= j
            disp(['mis-matching' states_all(j)]);
            disp([states_all(j) char(j)]);
        end
    end


    %# Set the initial states of the model
    editableCoordSet = osimModel.updCoordinateSet();
    for j = 1:size(states_all,1)
        str_addr = strfind(char(states_all(j)),'_u');
        if (str_addr & length(char(states_all(j)))-str_addr(end)==1)
            temp_str=char(states_all(j));
            editableCoordSet.get(temp_str(1:str_addr-1)).setSpeedValue(osimState, x0_temp(j));        
        else
            editableCoordSet.get(states_all(j)).setValue(osimState, x0_temp(j));
        end
    end
end


% Load the file that contain the initial guess for the controls (excitations)
[file_input, pathname] = uigetfile({'*.sto', 'OpenSim Controls (excitation) Files (*.sto)'}, ...
    'Select the initial controls file','MultiSelect', 'off');

if (file_input)
    cont_func = importdata(strcat(pathname,file_input)); % import controls data (1st column is time)

    old_time_u = cont_func.data(:,1);         % time 
    old_data_u = cont_func.data(:,2:end);     % the 6 states
    old_text_u = cont_func.textdata(7,2:end); % states names

    % Define a controller and add it to the model 
    muscleController = PrescribedController();
    muscleController.setName('PiecewiseLinear Controller')
    muscleController.setActuators(osimModel.updActuators())

    % get actuator set from model, and count the number of actuators
    model_act     = osimModel.getActuators();
    Nacts         = model_act.getSize();


    for i = 1:Nacts
       PLF = PiecewiseLinearFunction();
       % Get the coordinate set from the model
       CurrentActs = model_act.get(i-1);
       for j = 1:size(old_time_u,2)
           if strcmp(CurrentActs.getName(), old_text_u(j))
               for k=1:size(old_time_u,1)
                   PLF.addPoint(old_time_u(k),old_data_u(k,j));
               end
           end      
       end
       muscleController.prescribeControlForActuator(i-1,PLF);
    end

    osimModel.addController(muscleController);

end


% Call the function that runs the simulation
osimModel.setPropertiesFromState(osimState);

osimModel.equilibrateMuscles(osimState);


%  Save the Modified Model to a file
osimModel.print(fileoutpath);
disp(['The new model has been saved at ' fileoutpath]);




% Create a manager to run the simulation
simulationManager = Manager(osimModel);
simulationManager.setWriteToStorage(true);
simulationManager.setPerformAnalyses(true);
simulationManager.setInitialTime(TimeData.time_start);
simulationManager.setFinalTime(TimeData.time_end);
simulationManager.setIntegratorAccuracy(1e-04)


% start a timer
tic;

simulationManager.integrate(osimState);

% stop the timer
runtime = toc;
disp(['The runtime was ' num2str(runtime)]);


% Pull out the Pedal angles over the full simulation time
timeArray   = ArrayDouble();
B_Pedal_posArray = ArrayDouble();
G_Pedal_posArray = ArrayDouble();
SteerWh_posArray = ArrayDouble();

stateStorage = aStatesReporter.getStatesStorage();
stateStorage.getTimeColumn(timeArray,-1);
stateStorage.getDataColumn('B_Pedal_tilt',B_Pedal_posArray);
stateStorage.getDataColumn('G_Pedal_tilt',G_Pedal_posArray);
stateStorage.getDataColumn('Steering_Wh_ry',SteerWh_posArray);


AA = stateStorage.getColumnLabels;
AB = stateStorage.getLastStateVector().getData;
lastStateLabels = javaArray('java.lang.String',AB.getSize,1);
lastStateVals = zeros(AB.getSize,1);
for i=1:AB.getSize
    lastStateLabels(i,1) = AA.getitem(i);
    lastStateVals(i) = AB.getitem(i-1);
end


forceStorage = aForceReporter.getForceStorage;

AA = forceStorage.getColumnLabels;
AB = forceStorage.getLastStateVector().getData;
lastForceLabels = javaArray('java.lang.String',AB.getSize,1);
lastForceVals = zeros(AB.getSize,1);
for i=1:AB.getSize
    lastForceLabels(i,1) = AA.getitem(i);
    lastForceVals(i) = AB.getitem(i-1);
end



posStorage = aBodyKinematics.getPositionStorage();
velStorage = aBodyKinematics.getVelocityStorage();
accStorage = aBodyKinematics.getAccelerationStorage();

AA = posStorage.getColumnLabels;
AB = posStorage.getLastStateVector().getData;
BA = velStorage.getColumnLabels;
BB = velStorage.getLastStateVector().getData;
CA = accStorage.getColumnLabels;
CB = accStorage.getLastStateVector().getData;

lastBdKnPosLabels = javaArray('java.lang.String',AB.getSize,1);
lastBdKnPosVals = zeros(AB.getSize,1);
lastBdKnVelVals = zeros(BB.getSize,1);
lastBdKnAccVals = zeros(CB.getSize,1);
for i=1:AB.getSize
    lastBdKnPosLabels(i,1) = AA.getitem(i);
    lastBdKnPosVals(i) = AB.getitem(i-1);
    lastBdKnVelVals(i) = BB.getitem(i-1);
    lastBdKnAccVals(i) = CB.getitem(i-1);
end


kposStorage = aKinematics.getPositionStorage();
kvelStorage = aKinematics.getVelocityStorage();
kaccStorage = aKinematics.getAccelerationStorage();

AA = kposStorage.getColumnLabels;
AB = kposStorage.getLastStateVector().getData;
BA = kvelStorage.getColumnLabels;
BB = kvelStorage.getLastStateVector().getData;
CA = kaccStorage.getColumnLabels;
CB = kaccStorage.getLastStateVector().getData;

lastKinePosLabels = javaArray('java.lang.String',AB.getSize,1);
lastKinePosVals = zeros(AB.getSize,1);
lastKineVelVals = zeros(BB.getSize,1);
lastKineAccVals = zeros(CB.getSize,1);
for i=1:AB.getSize
    lastKinePosLabels(i,1) = AA.getitem(i);
    lastKinePosVals(i) = AB.getitem(i-1);
    lastKineVelVals(i) = BB.getitem(i-1);
    lastKineAccVals(i) = CB.getitem(i-1);
end

clear AA AB BA BB CA CB i j k;

% Write out storage files containing the states and excitations
outputpath = '.\Result\';  % use current directory
bufferstamp = char(datetime,'yy_MM_dd-HH_mm');
simulationManager.getStateStorage().print(['SimOut_',bufferstamp,'_states.sto']);
osimModel.printControlStorage(['SimOut_',bufferstamp,'_excitations.sto']);

% Also write out the results of the analyses to storage files
aActuation.printResults(['SimOut_',bufferstamp],outputpath,0.01,'.sto');
aBodyKinematics.printResults(['SimOut_',bufferstamp],outputpath,0.01,'.sto');
aForceReporter.printResults(['SimOut_',bufferstamp],outputpath,0.01,'.sto');
aJointReaction.printResults(['SimOut_',bufferstamp],outputpath,0.01,'.sto');
aKinematics.printResults(['SimOut_',bufferstamp],outputpath,0.01,'.sto');
aStatesReporter.printResults(['SimOut_',bufferstamp],outputpath,0.01,'.sto');

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
