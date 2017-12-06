function CentStruct = Init_Struct (pathname,Sys_Name)


    Model_Setting = readtable(strcat(pathname,Sys_Name,'_setting.txt'),'Delimiter','tab','ReadRowNames',false,'ReadVariableNames',true);
    Contt_Setting = readtable(strcat(pathname,Sys_Name,'_contact.txt'),'Delimiter','tab','ReadRowNames',true,'ReadVariableNames',false);
    % Create an OpenSim model from an OSIM file
    Model_In = [Sys_Name '.osim'];
    osimModel = Model(Model_In);

    output_Dir = 'Result';


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


    % Initialize the model (build the system and intialize the state)
    osimState = osimModel.initSystem();

    % Get the number of states from the model;
    % in this case the number of controls equals the number of muscles

    Num_Cent_Cnt = size(Model_Setting,1);
    Num_Cent_Contacts = size(Contt_Setting,1)-2;

    CentStruct.Model = osimModel;
    CentStruct.State = osimState;
    CentStruct.Number = Num_Cent_Cnt;
    CentStruct.TargetBody = cell(1, 1);
    CentStruct.TargetPointer = cell(1, 1);
    CentStruct.ContactNumber = Num_Cent_Contacts;
    CentStruct.TargetContact = cell(1, Num_Cent_Contacts);
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

    for i = 1:Num_Cent_Cnt
        CentStruct.Cent_cor_name{1,i} = Model_Setting.Coord{i};
        CentStruct.Cent_act_name{1,i} = Model_Setting.Act{i};
        if sum(strcmp(Model_Setting.Properties.VariableNames,'Des_Q'))
            CentStruct.Q_des(1,i) = Model_Setting.Des_Q(i) * pi / 180.;
        end
        if sum(strcmp(Model_Setting.Properties.VariableNames,'Des_U'))
            CentStruct.U_des(1,i) = Model_Setting.Des_U(i) * pi / 180.;
        end
    end



    for i=1:Num_Cent_Cnt
        Coord = osimModel.getCoordinateSet().get(CentStruct.Cent_cor_name{1,i});
        CentStruct.Q_ini(1,i) = Coord.getValue(osimState);
        CentStruct.U_ini(1,i) = Coord.getSpeedValue(osimState);
        Coord.setValue(osimState, 0.0);
        Coord.setSpeedValue(osimState, 0.0);
    end

    % % If I need to change initial joint angles, input values here.
    for i = 1:Num_Cent_Cnt
        if sum(strcmp(Model_Setting.Properties.VariableNames,'Init_Q'))
            CentStruct.Q_ini(1,i) = Model_Setting.Init_Q(i) * pi / 180.;
        end
        if sum(strcmp(Model_Setting.Properties.VariableNames,'Init_U'))
            CentStruct.U_ini(1,i) = Model_Setting.Init_U(i) * pi / 180.;
        end
    end


    % Solve difference of variable orders between osimModel and osimState.
    osimModel.realizeVelocity(osimState);
    for i=1:Num_Cent_Cnt
        Coord = osimModel.getCoordinateSet().get(CentStruct.Cent_cor_name{1,i});
        Coord.setValue(osimState, 1.);
        Coord.setSpeedValue(osimState, -8888.8);
        osimModel.realizeVelocity(osimState);
    %     tempQ = osimState.getQ;
        tempU = osimState.getU;
        for j=1:size(tempU)
            if tempU.get(j-1) == -8888.8
                CentStruct.ValAd(1,i) = j;
                break;
            end
        end
        Coord.setValue(osimState, 0.0);
        Coord.setSpeedValue(osimState, 0.0);
    end

    % Return initial state variables into osimModel.
    for i=1:Num_Cent_Cnt
        Coord = osimModel.getCoordinateSet().get(CentStruct.Cent_cor_name{1,i});
        Coord.setValue ( osimState, CentStruct.Q_ini(1,i) );
        Coord.setSpeedValue ( osimState, CentStruct.U_ini(1,i) );
    end


end