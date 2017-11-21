function Out_Struct = InitTargetStruct (pathname,Sys_Name)

    % Import the OpenSim modeling classes
    import org.opensim.modeling.*


    Model_Setting = readtable(strcat(pathname,Sys_Name,'_setting.txt'),'Delimiter','tab','ReadRowNames',false,'ReadVariableNames',true);
    Contt_Setting = readtable(strcat(pathname,Sys_Name,'_contact.txt'),'Delimiter','tab','ReadRowNames',false,'ReadVariableNames',true);
    % Create an OpenSim model from an OSIM file
    Model_In = [Sys_Name '.osim'];
    osimModel = Model(Model_In);

    output_Dir = 'Result';


    kp = 100;
    kv = 20;
    

    % Initialize the model (build the system and intialize the state)
    osimState = osimModel.initSystem();

    % Get the number of states from the model;
    % in this case the number of controls equals the number of muscles

    Num_Cont_Coord = size(Model_Setting,1);
    Num_Targets = size(Contt_Setting,1);
    Num_Contacts = size(Contt_Setting,2)-4;

    ContVarStruct = struct();
    ContVarStruct.Model = osimModel;
    ContVarStruct.State = osimState;
    ContVarStruct.outputdir = output_Dir;
    ContVarStruct.Number = Num_Cont_Coord;
    ContVarStruct.TargetBody = cell(Num_Targets,1);
    ContVarStruct.TargetSwitch = cell(Num_Targets,1);
    ContVarStruct.Target_RelCoord = cell(Num_Targets,1);
    ContVarStruct.TargetSub_Pointer = cell(Num_Targets,1);
    ContVarStruct.TargetDes_Pointer = cell(Num_Targets,1);
    ContVarStruct.ContactNumber = Num_Contacts;
    ContVarStruct.TargetContact = cell(Num_Targets, Num_Contacts);
    ContVarStruct.Cent_val_name = cell(1, Num_Cont_Coord);
    ContVarStruct.Cent_cor_name = cell(1, Num_Cont_Coord);
    ContVarStruct.Cent_act_name = cell(1, Num_Cont_Coord);
    ContVarStruct.ValAd = zeros(1, Num_Cont_Coord);
    ContVarStruct.Q_des = zeros(1, Num_Cont_Coord);
    ContVarStruct.U_des = zeros(1, Num_Cont_Coord);
    ContVarStruct.Q_ini = zeros(1, Num_Cont_Coord);
    ContVarStruct.U_ini = zeros(1, Num_Cont_Coord);
    ContVarStruct.ContV = zeros(1, Num_Cont_Coord);
    ContVarStruct.kp = kp * ones(1, Num_Cont_Coord);
    ContVarStruct.kv = kv * ones(1, Num_Cont_Coord);

    for i = 1:Num_Cont_Coord
        ContVarStruct.Cent_cor_name{1,i} = Model_Setting.Coord{i};
        ContVarStruct.Cent_act_name{1,i} = Model_Setting.Act{i};
        if sum(strcmp(Model_Setting.Properties.VariableNames,'Des_Q'))
            ContVarStruct.Q_des(1,i) = Model_Setting.Des_Q(i) * pi / 180.;
        end
        if sum(strcmp(Model_Setting.Properties.VariableNames,'Des_U'))
            ContVarStruct.U_des(1,i) = Model_Setting.Des_U(i) * pi / 180.;
        end
        if sum(strcmp(Model_Setting.Properties.VariableNames,'kp'))
            if Model_Setting.kp(i)
                ContVarStruct.kp(1,i) = Model_Setting.kp(i);
            end
        end
        if sum(strcmp(Model_Setting.Properties.VariableNames,'kv'))
            if Model_Setting.kv(i)
                ContVarStruct.kv(1,i) = Model_Setting.kv(i);
            end
        end
    end



    for i=1:Num_Cont_Coord
        Coord = osimModel.getCoordinateSet().get(ContVarStruct.Cent_cor_name{1,i});
        ContVarStruct.Q_ini(1,i) = Coord.getValue(osimState);
        ContVarStruct.U_ini(1,i) = Coord.getSpeedValue(osimState);
        Coord.setValue(osimState, 0.0);
        Coord.setSpeedValue(osimState, 0.0);
    end

    % % If I need to change initial joint angles, input values here.
    for i = 1:Num_Cont_Coord
        if sum(strcmp(Model_Setting.Properties.VariableNames,'Init_Q'))
            ContVarStruct.Q_ini(1,i) = Model_Setting.Init_Q(i) * pi / 180.;
        end
        if sum(strcmp(Model_Setting.Properties.VariableNames,'Init_U'))
            ContVarStruct.U_ini(1,i) = Model_Setting.Init_U(i) * pi / 180.;
        end
    end


    % Solve difference of variable orders between osimModel and osimState.
    osimModel.realizeVelocity(osimState);
    for i=1:Num_Cont_Coord
        Coord = osimModel.getCoordinateSet().get(ContVarStruct.Cent_cor_name{1,i});
        Coord.setValue(osimState, 1.);
        Coord.setSpeedValue(osimState, -8888.8);
        osimModel.realizeVelocity(osimState);
    %     tempQ = osimState.getQ;
        tempU = osimState.getU;
        for j=1:size(tempU)
            if tempU.get(j-1) == -8888.8
                ContVarStruct.ValAd(1,i) = j;
                break;
            end
        end
        Coord.setValue(osimState, 0.0);
        Coord.setSpeedValue(osimState, 0.0);
    end

    % Return initial state variables into osimModel.
    for i=1:Num_Cont_Coord
        Coord = osimModel.getCoordinateSet().get(ContVarStruct.Cent_cor_name{1,i});
        Coord.setValue ( osimState, ContVarStruct.Q_ini(1,i) );
        Coord.setSpeedValue ( osimState, ContVarStruct.U_ini(1,i) );
    end

    for i = 1:Num_Targets
        ContVarStruct.TargetBody{i,1} = Contt_Setting.TargetBody{i,1};
        ContVarStruct.TargetSwitch{i,1} = Contt_Setting.TargetSwitch{i,1};
        ContVarStruct.TargetSub_Pointer{i,1} = Contt_Setting.TargetSub_Pointer{i,1};
        ContVarStruct.TargetDes_Pointer{i,1} = Contt_Setting.TargetDes_Pointer{i,1};
        for j = 1:Num_Contacts;
            ContVarStruct.TargetContact{i,j} = char(Contt_Setting{i,j+4});
        end
        for j = 1:osimModel.getCoordinateSet.getSize
            if strcmp(osimModel.getBodySet.get(osimModel.getCoordinateSet.get(j-1).getBodyIndex).getName,ContVarStruct.TargetBody{i,1})
                ContVarStruct.Target_RelCoord{i,1} = char(osimModel.getCoordinateSet.get(j-1).getName);
                break;
            end
        end
    end
   
    Out_Struct = ContVarStruct;

end