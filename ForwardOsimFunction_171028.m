function ForwardOsimFunction_171028(time_start,time_end,ExtraPar,InitStruct)
% ----------------------------------------------------------------------- %
% ForwardOsimFunction_YYMMDD.m
%
% Author: Hideyuki Kimpara, Worcester Polytechnic Institute
%
% This code uses the Matlab interface to the OpenSim API to run a trial
% to connect simulink simulator and OpenSim analysis. 
% This example borrows from other Matlab and C++ examples avaiable from
% the OpenSim project written by several people including, but not limited
% to: Brian Umberger and OpenSim team members at Stanford University.
% ----------------------------------------------------------------------- %

    % Import the OpenSim modeling classes
    import org.opensim.modeling.*

    % Get reference to the model
    osimModel       = ExtraPar.model;
%     s               = ExtraPar.state;
    CentStruct      = ExtraPar.CentSt;
    Control_Func       = ExtraPar.Control;
    aJointReaction = ExtraPar.JointReact;



    % Check to see if model state is initialized by checking size
    if(osimModel.getWorkingState().getNY() == 0)
       s = osimModel.initSystem();
    else
       s = osimModel.updWorkingState(); 
    end
    
    %# Set the initial states of the model
    editableCoordSet = osimModel.updCoordinateSet();

    % Arrange the initial guess by nodes and states
    for ii = 1:size(InitStruct.StateVals,1) 
        VariableName = char(InitStruct.StateLabl(ii));
        SpString = split(InitStruct.StateLabl(ii),'/');
        JointName = char(SpString(1));
        CoordName = char(SpString(2));
        ValueName = char(SpString(3));
        if strcmp(ValueName,'speed')
            temp_str=char(InitStruct.StateLabl(ii));
            editableCoordSet.get(CoordName).setSpeedValue(s, InitStruct.StateVals(ii));        
        else
            editableCoordSet.get(CoordName).setValue(s, InitStruct.StateVals(ii));
        end
    end

    clear editableCoordSet temp_str str_addr;

%     for ii=1:CentStruct.Number
%     end

    joint_1 = osimModel.getCoordinateSet().get(CentStruct.Cent_cor_name(1,1));
    joint_2 = osimModel.getCoordinateSet().get(CentStruct.Cent_cor_name(1,2));
    
     q1 = joint_1.getValue(s);
    dq1 = joint_1.getSpeedValue(s);
     q2 = joint_2.getValue(s);
    dq2 = joint_2.getSpeedValue(s);

     q1_des =  CentStruct.Q_des(1,1);
    dq1_des =  CentStruct.U_des(1,1);
     q2_des =  CentStruct.Q_des(1,2);
    dq2_des =  CentStruct.U_des(1,2);
    
%     disp(['joint1 vl: [' num2str(q1/pi*180.) ']/desired[' num2str(q1_des/pi*180.) '] ']);
%     disp(['joint1 sp: [' num2str(dq1/pi*180.) ']/desired[' num2str(dq1_des/pi*180.) '] ']);
%     disp(['joint2 vl: [' num2str(q2/pi*180.) ']/desired[' num2str(q2_des/pi*180.) '] ']);
%     disp(['joint2 sp: [' num2str(dq2/pi*180.) ']/desired[' num2str(dq2_des/pi*180.) '] ']);
    
    % controlLaw = Vector( 2, 0.0 );
    q = zeros(1,2); %   q = Vector( 2, 0.0 );
    q(1) = q1; q(2) = q2; % q.set(0, q1); q.set(1, q2);
    dq = zeros(1,2); % dq = Vector( 2, 0.0 );
    dq(1) = dq1; dq(2) = dq2; % dq.set(0, dq1); dq.set(1, dq2);
    q_des = zeros(1,2); % q_des = Vector( 2, 0.0 );
    q_des(1) = q1_des; q_des(2) = q2_des; % q_des.set(0, q1_des); q_des.set(1, q2_des);
    dq_des = zeros(1,2); % dq_des = Vector( 2, 0.0 );
    dq_des(1) = dq1_des; dq_des(2) = dq2_des; % dq_des.set(0, dq1_des); dq_des.set(1, dq2_des);
    
    KP = diag(CentStruct.kp(1,1:2)); 
    KV = diag(CentStruct.kv(1,1:2)); % KP = Matrix( 2, 2, 0.0 ); KV = Matrix( 2, 2, 0.0 );
    % KP.set( 0, 0, kp ); KP.set( 1, 1, kp );
    % KV.set( 0, 0, kv ); KV.set( 1, 1, kv );
    
    controlLaw = zeros(1,CentStruct.Number);
    
    controlLaw = (-1. * KP*(q - q_des)' - KV*(dq - dq_des)')';
    
    controlLaw_Full = zeros(1,s.getNU);
    controlLaw_Vector = Vector(s.getNU(), 0.0);
    
    for ii=1:CentStruct.Number
        controlLaw_Full(1,CentStruct.ValAd(1,ii)) = controlLaw(1,ii);
        controlLaw_Vector.set(CentStruct.ValAd(1,ii)-1, controlLaw(1,ii));
    end
    
    smss = osimModel.getMatterSubsystem();

    osimModel.realizeVelocity(s);
    
    % Calculate MUDot = M * Udot
    MUDot = Vector();
    smss.multiplyByM(s, controlLaw_Vector, MUDot);
    
    M = Matrix(s.getNU(),s.getNU());
    smss.calcM(s,M);
    M_array = osimMatrixToArray(M);
    MUDot_array = (M_array * controlLaw_Full')';

    % Calculate G_vector (Gravity force vector)
    G_vector = Vector(s.getNU(),0.0);
    GrvForce = osimModel.getGravityForce();

%     % ****[ POINT 1 ]****
%     % ** This method is not wrapped for MATLAB **
%     % G_vector = GrvForce.getBodyForces(s);

    % Calculate Cq_vector (Velocity vector)
    Cq_vector = Vector();
    knownUdot = Vector(s.getNU(), 0.0);
    appliedMobilityForces = Vector();
    appliedBodyForces = VectorOfSpatialVec();
    
    smss.calcResidualForceIgnoringConstraints(s, appliedMobilityForces, appliedBodyForces, ...
        knownUdot, Cq_vector);
    
    
    % Test SimbodyMatterSubsystem.calcResidualForce().
    % ------------------------------------------------
    % For the given inputs, we will actually be computing the first column
    % of the mass matrix. We accomplish this by setting all inputs to 0
    % except for the acceleration of the first coordinate.
    %   f_residual = M udot + f_inertial + f_applied 
    %              = M ~[1, 0, ...] + 0 + 0
    
    knownLambda = Vector();
    residualMobilityForces = Vector();
    smss.calcResidualForce(s, Cq_vector, appliedBodyForces, ...
                  controlLaw_Vector, knownLambda, residualMobilityForces);
    
    % Calculate Joint torques
                  
    controlTorque = Vector();

    G_array = osimVectorToArray(G_vector);
    Cq_array = osimVectorToArray(Cq_vector);
    
    assert (size(MUDot_array,2) == size(Cq_array,2));
    
    controlTorque_array = MUDot_array + Cq_array - G_array;
    
%     FSS = osimModel.getForceSet();
%     FSS1 = FSS.get(1);
%     FSS1.getRecordValues(osimState)
%     FSS
%     FSS1.getName
%     FSS.getSize
%     FSS.getName
%     FSS.getName(1)
%     FSS.get(1).getName    
    
    for ii=1:size(controlTorque_array,2)
        if controlTorque_array(1,ii)>400.
            controlTorque_array(1,ii) = 400.;
        elseif controlTorque_array(1,ii)<-400.
            controlTorque_array(1,ii) = -400.;
        end
    end

    controlTorque = osimVectorFromArray(controlTorque_array);

    disp(['controlTorque: [' num2str(controlTorque_array) ']' ]);
    
    for jj=1:CentStruct.Number
        CentStruct.ContV(jj) = controlTorque.get(CentStruct.ValAd(1,jj)-1);
    end
    
    assert(controlTorque.size() == s.getNU());

    
    for ii=1:size(Control_Func.time,2)
        if Control_Func.time(ii)>time_start && Control_Func.time(ii)<=time_end
            for jj=1:CentStruct.Number
                assert(strcmp (Control_Func.name(jj), CentStruct.Cent_act_name(jj)));
                Control_Func.data(jj, ii) = CentStruct.ContV(jj);
            end
        end
    end


    % Delete old ControllerSet from the model 
    N_olderCont = osimModel.getControllerSet().getSize();
    for ii = 1: N_olderCont 
        olderController = osimModel.getControllerSet().get(ii-1);
        osimModel.removeController(olderController);
        clear olderController
    end

    clear N_olderCont 

%      q1m = joint_1.getValue(s);
%     dq1m = joint_1.getSpeedValue(s);
%      q2m = joint_2.getValue(s);
%     dq2m = joint_2.getSpeedValue(s);
% 
%     disp(['Check Point A']);
%     disp(['joint1 vl: [' num2str(q1m/pi*180.) ']/desired[' num2str(q1_des/pi*180.) '] ']);
%     disp(['joint1 sp: [' num2str(dq1m/pi*180.) ']/desired[' num2str(dq1_des/pi*180.) '] ']);
%     disp(['joint2 vl: [' num2str(q2m/pi*180.) ']/desired[' num2str(q2_des/pi*180.) '] ']);
%     disp(['joint2 sp: [' num2str(dq2m/pi*180.) ']/desired[' num2str(dq2_des/pi*180.) '] ']);


    % Define a controller and add it to the model 
    ActuatorController = PrescribedController();
    ActuatorController.setName('PiecewiseLinear Controller')
    ActuatorController.setActuators(osimModel.updActuators())

    % get actuator set from model, and count the number of actuators
    model_act     = osimModel.getActuators();
    Nacts         = model_act.getSize();


    for ii = 1:Nacts
       PLF = PiecewiseLinearFunction();
       % Get the coordinate set from the model
       CurrentActs = model_act.get(ii-1);
       for jj = 1:size(Control_Func.name,2)
           if strcmp(CurrentActs.getName(), Control_Func.name(jj))
               for kk=1:size(Control_Func.time,2)
                   PLF.addPoint(Control_Func.time(kk),Control_Func.data(jj,kk));
               end
               break;
           end      
       end
       ActuatorController.prescribeControlForActuator(ii-1,PLF);
    end

    clear PLF CurrentActs model_act Nacts;

%     % ****[ POINT 2 ]****
%     % Add controller to osimModel
    osimModel.addController(ActuatorController);


    clear muscleController;

    
%     %  Save the Modified Model to a file
%     fileoutpath = ['Monitor_Model.osim'];
%     osimModel.print(fileoutpath);
%     disp(['The new model has been saved at ' fileoutpath]);
%     % 

        
%     % ****[ POINT 3 ]****
%     % Call the function that runs the simulation
%     osimModel.setPropertiesFromState(s);
%     osimModel.equilibrateMuscles(s);


    osimModel.initSystem();

    %# Set the initial states of the model
    editableCoordSet = osimModel.updCoordinateSet();

    % Arrange the initial guess by nodes and states
    for ii = 1:size(InitStruct.StateVals,1) 
        VariableName = char(InitStruct.StateLabl(ii));
        SpString = split(InitStruct.StateLabl(ii),'/');
        JointName = char(SpString(1));
        CoordName = char(SpString(2));
        ValueName = char(SpString(3));
        if strcmp(ValueName,'speed')
            temp_str=char(InitStruct.StateLabl(ii));
            editableCoordSet.get(CoordName).setSpeedValue(s, InitStruct.StateVals(ii));        
        else
            editableCoordSet.get(CoordName).setValue(s, InitStruct.StateVals(ii));
        end
    end

    clear editableCoordSet temp_str str_addr;

    
    
    % Create a manager to run the simulation
    simulationManager = Manager(osimModel);
    simulationManager.setWriteToStorage(true);
    simulationManager.setPerformAnalyses(true);
    simulationManager.setInitialTime(time_start);
    simulationManager.setFinalTime(time_end);
    simulationManager.setIntegratorAccuracy(1e-04);
    
    simulationManager.integrate(s);

%      q1 = joint_1.getValue(s);
%     dq1 = joint_1.getSpeedValue(s);
%      q2 = joint_2.getValue(s);
%     dq2 = joint_2.getSpeedValue(s);
% 
%      q1_des =  CentStruct.Q_des(1,1);
%     dq1_des =  CentStruct.U_des(1,1);
%      q2_des =  CentStruct.Q_des(1,2);
%     dq2_des =  CentStruct.U_des(1,2);
%     
%     disp(['Check Point A']);
%     disp(['joint1 vl: [' num2str(q1/pi*180.) ']/desired[' num2str(q1_des/pi*180.) '] ']);
%     disp(['joint1 sp: [' num2str(dq1/pi*180.) ']/desired[' num2str(dq1_des/pi*180.) '] ']);
%     disp(['joint2 vl: [' num2str(q2/pi*180.) ']/desired[' num2str(q2_des/pi*180.) '] ']);
%     disp(['joint2 sp: [' num2str(dq2/pi*180.) ']/desired[' num2str(dq2_des/pi*180.) '] ']);
    
    osimModel.realizeDynamics(s);
    ForceSet = osimModel.getForceSet();
    Contact1 = ForceSet.get('Contact1');
    Contact_Forces = Contact1.getRecordValues(s);
    disp(['contact forces = ' char(Contact_Forces)]);
    
    
    simulationManager.delete;
    clear simulationManager;

end
