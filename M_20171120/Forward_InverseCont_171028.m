function [Output_StateVals, Output_Forces, Output_Controls] = Forward_InverseCont_171028(time_start,time_end,ExtraPar,InitStruct,Comp_KPs,Comp_KV)
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
    TargetVar_Struct      = ExtraPar.TargetVar_Struct;
    Control_Func       = ExtraPar.Control;
    Reporter_Struct    = ExtraPar.Reporter_Struct;
%     aJointReaction = ExtraPar.JointReact;

    % Check to see if model state is initialized by checking size
    if(osimModel.getWorkingState().getNY() == 0)
       s = osimModel.initSystem();
    else
       s = osimModel.updWorkingState(); 
    end
    
%     osimModel.initSystem();
    
    %# Set the initial states of the model
    editableCoordSet = osimModel.updCoordinateSet();

    % Arrange the initial guess by nodes and states
    for ii = 1:size(InitStruct.StateVals,1) 
        VariableName = char(InitStruct.StateLabl{ii,1});
        SpString = split(InitStruct.StateLabl{ii,1},'/');
        JointName = char(SpString(1));
        CoordName = char(SpString(2));
        ValueName = char(SpString(3));
        if strcmp(ValueName,'speed')
            editableCoordSet.get(CoordName).setSpeedValue(s, InitStruct.StateVals(ii));        
        else
            editableCoordSet.get(CoordName).setValue(s, InitStruct.StateVals(ii));
        end
    end

    clear editableCoordSet temp_str str_addr;


    for i = 1:length(TargetVar_Struct.TargetBody)
        if strcmp(TargetVar_Struct.Target_current,TargetVar_Struct.TargetBody{i})
            Target_adr = i;
            break;
        end
    end
    
    TarSubBD = osimModel.getBodySet().get(TargetVar_Struct.TargetSub_Pointer{Target_adr});
    position = osimVec3ToArray( TarSubBD.getPositionInGround(s) );
        
    KP = eye(3);
    for i = 1:3
        KP(i,i) = KP(i,i)*Comp_KPs(i);
    end
    
    KV = eye(TargetVar_Struct.Number);
    dp = zeros(1,TargetVar_Struct.Number);
    for i = 1:TargetVar_Struct.Number
%         KV(i,i) = KV(i,i)*TargetVar_Struct.kv(i);
        KV(i,i) = KV(i,i)*Comp_KV;
        dp(i) = osimModel.getCoordinateSet().get(TargetVar_Struct.Cent_cor_name{i}).getSpeedValue(s);
    end 
    
    control = KP * (TargetVar_Struct.P_des - position)';

%     disp(['[Distance] : ' num2str(norm(TargetVar_Struct.P_des - position))  ', val: [' num2str(position) ']/desired[' num2str(TargetVar_Struct.P_des) '] ']);
    
    JT_array = getStationJacobian(osimModel, s, TarSubBD.getMobilizedBodyIndex);
    Tau1 = JT_array * control;
    
    Tau2 = zeros(length(Tau1),1);
    Tau2a = KV*dp';
    
    for i = 1:TargetVar_Struct.Number
        Tau2(TargetVar_Struct.ValAd(i)) = Tau2a(i);
%         disp([TargetVar_Struct.Cent_cor_name{1,i} ' val: [' ...
%             num2str(osimModel.getCoordinateSet.get(TargetVar_Struct.Cent_cor_name{1,i}).getValue(s)/pi*180.)...
%             '] and spd: [' num2str(osimModel.getCoordinateSet.get(TargetVar_Struct.Cent_cor_name{1,i}).getSpeedValue(s)/pi*180.)  ']']);
    end
    
    % Calculate G_vector (Gravity force vector)
    G_vector = Vector(s.getNU(),0.0);

    osimModel.getMatterSubsystem().multiplyBySystemJacobianTranspose (s,...
        SimbodyUtils.getGravityForce_getBodyForces ( osimModel, s ), G_vector);

    G_array = (osimVectorToArray(G_vector))';

    Tau = zeros(TargetVar_Struct.Number,1);
    for i = 1:TargetVar_Struct.Number
%         Tau(i) = Tau1(TargetVar_Struct.ValAd(i)) - Tau2a(i) - G_array(TargetVar_Struct.ValAd(i));
        Tau(i) = + Tau1(TargetVar_Struct.ValAd(i)) - Tau2a(i);
    end
  
    controlTorque_array = Tau';
    
%     disp(['[torques] : ' num2str(controlTorque_array)]);
   
    Target_address = getTargetCurID(TargetVar_Struct);
%     if strfind(TargetVar_Struct.Target_current,'G_Pedal')
%         for i = 1:size(TargetVar_Struct.TargetBody,1)
%             if strfind(TargetVar_Struct.TargetBody{i,1},'G_Pedal')
%                 Target_address = i;
%                 break;
%             end
%         end
%     else
%         for i = 1:size(TargetVar_Struct.TargetBody,1)
%             if strfind(TargetVar_Struct.TargetBody{i,1},'B_Pedal')
%                 Target_address = i;
%                 break;
%             end
%         end
%     end
    
    


    
    
    for ii=1:length(controlTorque_array)
        if controlTorque_array(ii)>400.
            controlTorque_array(ii) = 400.;
        elseif controlTorque_array(ii)<-400.
            controlTorque_array(ii) = -400.;
        end
    end

    controlTorque = osimVectorFromArray(controlTorque_array);

%     disp(['controlTorque: [' num2str(controlTorque_array) ']' ]);
    
    for jj=1:TargetVar_Struct.Number
        TargetVar_Struct.ContV(jj) = controlTorque.get(jj-1);
    end
    
    
    for ii=1:size(Control_Func.time,2)
        if Control_Func.time(ii)>time_start && Control_Func.time(ii)<=time_end
            for jj=1:TargetVar_Struct.Number
                assert(strcmp (Control_Func.name{jj,1}, TargetVar_Struct.Actuator_name{1,jj}));
                Control_Func.data(jj, ii) = TargetVar_Struct.ContV(jj);
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
       for jj = 1:length(Control_Func.name)
           if strcmp(CurrentActs.getName(), Control_Func.name{jj,1})
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
        VariableName = char(InitStruct.StateLabl{ii,1});
        SpString = split(InitStruct.StateLabl{ii,1},'/');
        JointName = char(SpString(1));
        CoordName = char(SpString(2));
        ValueName = char(SpString(3));
        if strcmp(ValueName,'speed')
            temp_str=char(InitStruct.StateLabl{ii,1});
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
%      q1_des =  TargetVar_Struct.Q_des(1,1);
%     dq1_des =  TargetVar_Struct.U_des(1,1);
%      q2_des =  TargetVar_Struct.Q_des(1,2);
%     dq2_des =  TargetVar_Struct.U_des(1,2);
%     
%     disp(['Check Point A']);
%     disp(['joint1 vl: [' num2str(q1/pi*180.) ']/desired[' num2str(q1_des/pi*180.) '] ']);
%     disp(['joint1 sp: [' num2str(dq1/pi*180.) ']/desired[' num2str(dq1_des/pi*180.) '] ']);
%     disp(['joint2 vl: [' num2str(q2/pi*180.) ']/desired[' num2str(q2_des/pi*180.) '] ']);
%     disp(['joint2 sp: [' num2str(dq2/pi*180.) ']/desired[' num2str(dq2_des/pi*180.) '] ']);
    
    osimModel.realizeDynamics(s);

    %# Set the initial states of the model
    CurrentCoordSet = osimModel.getCoordinateSet();
    
    UpdatedStateVals = zeros(size(InitStruct.StateLabl,1),1);
    % Arrange the initial guess by nodes and states
    for ii = 1:size(InitStruct.StateVals,1) 
        VariableName = char(InitStruct.StateLabl{ii,1});
        SpString = split(InitStruct.StateLabl{ii,1},'/');
        JointName = char(SpString(1));
        CoordName = char(SpString(2));
        ValueName = char(SpString(3));
        if strcmp(ValueName,'speed')
            UpdatedStateVals(ii) = CurrentCoordSet.get(CoordName).getSpeedValue(s);        
        else
            UpdatedStateVals(ii) = CurrentCoordSet.get(CoordName).getValue(s);
        end
    end

    Output_StateVals = UpdatedStateVals;
    
    smss = osimModel.getMatterSubsystem();
    JointRF = VectorOfSpatialVec();
  %  smss.calcMobilizerReactionForces(s, JointRF);
    
    
    ForceSet = osimModel.getForceSet();
    
%     OutForces = zeros(1,6);
%     
%     for ii = 1:TargetVar_Struct.ContactNumber
%         counter = 1;
%         ContactSet = ForceSet.get(TargetVar_Struct.TargetContact{Target_address,ii});
%         for jj=1:ContactSet.getRecordLabels().getSize
%             if (strfind(ContactSet.getRecordLabels().get(jj-1),TargetVar_Struct.TargetBody{Target_address,1})>0)
%                 OutForces(counter) = OutForces(counter) + ContactSet.getRecordValues(s).get(jj-1);
%                 counter = counter + 1;
%                 if counter>6
%                     break;
%                 end
%             end
%         end
%     end
    OutForces = zeros(Reporter_Struct.contact_report.number*8, 1);

    for i = 1:Reporter_Struct.contact_report.number
        for j = 1:ForceSet.getSize
            if strcmp(ForceSet.get(j-1).getName(),Reporter_Struct.contact_report.contact{1,i})
                Cur_Force = ForceSet.get(j-1);
                Count = 0;
                for k = 1:Cur_Force.getRecordLabels().getSize
                    for ii = 1:6
                        if strfind(Cur_Force.getRecordLabels().get(k-1),Reporter_Struct.contact_report.dataLabls{(i-1)*8+ii,1})
                            OutForces((i-1)*8+ii,1) = Cur_Force.getRecordValues(s).get(k-1);
                            Count = Count + 1;
                        end
                    end
                    if Count >= 6
                        Count = Count + 1;
                        OutForces((i-1)*8+7,1)...
                            = sqrt( OutForces((i-1)*8+1,1)^2+OutForces((i-1)*8+2,1)^2+OutForces((i-1)*8+3,1)^2 );
                        Count = Count + 1;
                        OutForces((i-1)*8+8,1)...
                            = sqrt( OutForces((i-1)*8+4,1)^2+OutForces((i-1)*8+5,1)^2+OutForces((i-1)*8+6,1)^2 );
                        break;
                    end
                end
                if Count
                    break;
                end
            end
        end
    end
    

    Output_Forces = OutForces;
    
    Output_Controls = Control_Func.data;
%     disp_string = sprintf('contact forces on %s = [ ',TargetVar_Struct.TargetBody{Target_address,1});
%     for ii = 1:6
%         disp_string = char([disp_string num2str(Output_ExtForces.Force(ii)) ', ']);
%     end
%     disp_string = char([disp_string ' ]']);
%     disp(disp_string);
%     disp(' ');
    
    
    simulationManager.delete;
    clear simulationManager;
    
end


function    Out_Matrix_array = getStationJacobian(osimModel, s, MobilizedBodyIndex)

    % Import the OpenSim modeling classes
    import org.opensim.modeling.*

    station = Vec3(0);
    J = Matrix();
    
    osimModel.getMatterSubsystem().calcStationJacobian(s, MobilizedBodyIndex, station, J);
    J_array = osimMatrixToArray(J);
    Out_Matrix_array = J_array';
    
end