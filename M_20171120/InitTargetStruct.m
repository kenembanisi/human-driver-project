function Out_Struct = InitTargetStruct (pathname,Sys_Name)

    % Import the OpenSim modeling classes
    import org.opensim.modeling.*

    if str2num(char(org.opensim.modeling.opensimCommon.GetVersion())) < 4.0
        disp(['Please install OpenSim 4.0 or greater than 4.0']);
        exit(-1);
    end

    pause(1);


    Model_Setting = readtable(strcat(pathname,Sys_Name,'_setting.txt'),'Delimiter','tab','ReadRowNames',false,'ReadVariableNames',true);
    Contt_Setting = readtable(strcat(pathname,Sys_Name,'_contact.txt'),'Delimiter','tab','ReadRowNames',false,'ReadVariableNames',true);
    % Create an OpenSim model from an OSIM file
    Model_In = [Sys_Name '.osim'];
    osimModel = Model(Model_In);

    output_Dir = 'Result';
    bufferstamp = char(datetime,'yy_MM_dd-HH_mm');


    kp = 100;
    kv = 20;
    

    % Initialize the model (build the system and intialize the state)
    osimState = osimModel.initSystem();

    % Get the number of states from the model;
    % in this case the number of controls equals the number of muscles

    Num_Cont_Coord = size(Model_Setting,1);
    Num_Targets = size(Contt_Setting,1);
    Num_Contacts = size(Contt_Setting,2)-4;

    TargetVar_Struct = struct();
    TargetVar_Struct.Model = osimModel;
    TargetVar_Struct.State = osimState;
    TargetVar_Struct.pathname= pathname;
    TargetVar_Struct.SysName = Sys_Name;
    TargetVar_Struct.outputdir = output_Dir;
    TargetVar_Struct.bufferstamp = bufferstamp;
    TargetVar_Struct.Number = Num_Cont_Coord; % number of applied actuators
    TargetVar_Struct.TargetBody = cell(Num_Targets,1);
    TargetVar_Struct.TargetSwitch = cell(Num_Targets,1); 
    TargetVar_Struct.Target_RelCoord = cell(Num_Targets,1); % this holds the name of the pedal joint coordinate
    TargetVar_Struct.TargetSub_Pointer = cell(Num_Targets,1); % pointer body name -- pointer_Toe
    TargetVar_Struct.TargetDes_Pointer = cell(Num_Targets,1); % tracked body name -- pointer_Gas & brake
    TargetVar_Struct.ContactNumber = Num_Contacts; % number of contact interfaces we are interested in
    TargetVar_Struct.TargetContact = cell(Num_Targets, Num_Contacts); 
    TargetVar_Struct.Cent_val_name = cell(1, Num_Cont_Coord);
    TargetVar_Struct.Cent_cor_name = cell(1, Num_Cont_Coord);
    TargetVar_Struct.Actuator_name = cell(1, Num_Cont_Coord);
    TargetVar_Struct.Actuator_Data = zeros(1, Num_Cont_Coord);
    TargetVar_Struct.Actuator_Time = round(osimState.getTime,4);
    TargetVar_Struct.ValAd         = zeros(1, Num_Cont_Coord);
    TargetVar_Struct.P_des         = zeros(1,3);
    TargetVar_Struct.Pdot_des      = zeros(1,3);
    TargetVar_Struct.Q_des         = zeros(1, Num_Cont_Coord);
    TargetVar_Struct.U_des         = zeros(1, Num_Cont_Coord);
    TargetVar_Struct.Q_ini         = zeros(1, Num_Cont_Coord);
    TargetVar_Struct.U_ini         = zeros(1, Num_Cont_Coord);
    TargetVar_Struct.kp            = kp * ones(1, Num_Cont_Coord);
    TargetVar_Struct.kv            = kv * ones(1, Num_Cont_Coord);

    for i = 1:Num_Cont_Coord
        TargetVar_Struct.Cent_cor_name{1,i} = Model_Setting.Coord{i};
        TargetVar_Struct.Actuator_name{1,i} = Model_Setting.Act{i};
        if sum(strcmp(Model_Setting.Properties.VariableNames,'Des_Q'))
            TargetVar_Struct.Q_des(1,i) = Model_Setting.Des_Q(i) * pi / 180.;
        end
        if sum(strcmp(Model_Setting.Properties.VariableNames,'Des_U'))
            TargetVar_Struct.U_des(1,i) = Model_Setting.Des_U(i) * pi / 180.;
        end
        if sum(strcmp(Model_Setting.Properties.VariableNames,'kp'))
            if Model_Setting.kp(i)
                TargetVar_Struct.kp(1,i) = Model_Setting.kp(i);
            end
        end
        if sum(strcmp(Model_Setting.Properties.VariableNames,'kv'))
            if Model_Setting.kv(i)
                TargetVar_Struct.kv(1,i) = Model_Setting.kv(i);
            end
        end
    end



    for i=1:Num_Cont_Coord
        Coord = osimModel.getCoordinateSet().get(TargetVar_Struct.Cent_cor_name{1,i});
        TargetVar_Struct.Q_ini(1,i) = Coord.getValue(osimState);
        TargetVar_Struct.U_ini(1,i) = Coord.getSpeedValue(osimState);
        Coord.setValue(osimState, 0.0);
        Coord.setSpeedValue(osimState, 0.0);
    end

    % % If I need to change initial joint angles, input values here.
    for i = 1:Num_Cont_Coord
        if sum(strcmp(Model_Setting.Properties.VariableNames,'Init_Q'))
            TargetVar_Struct.Q_ini(1,i) = Model_Setting.Init_Q(i) * pi / 180.;
        end
        if sum(strcmp(Model_Setting.Properties.VariableNames,'Init_U'))
            TargetVar_Struct.U_ini(1,i) = Model_Setting.Init_U(i) * pi / 180.;
        end
    end


    % Solve difference of variable orders between osimModel and osimState.
    osimModel.realizeVelocity(osimState);
    for i=1:Num_Cont_Coord
        Coord = osimModel.getCoordinateSet().get(TargetVar_Struct.Cent_cor_name{1,i});
        Coord.setValue(osimState, 1.);
        Coord.setSpeedValue(osimState, -8888.8);
        osimModel.realizeVelocity(osimState);
    %     tempQ = osimState.getQ;
        tempU = osimState.getU;
        for j=1:size(tempU)
            if tempU.get(j-1) == -8888.8
                TargetVar_Struct.ValAd(1,i) = j;
                break;
            end
        end
        Coord.setValue(osimState, 0.0);
        Coord.setSpeedValue(osimState, 0.0);
    end

    % Return initial state variables into osimModel.
    for i=1:Num_Cont_Coord
        Coord = osimModel.getCoordinateSet().get(TargetVar_Struct.Cent_cor_name{1,i});
        Coord.setValue ( osimState, TargetVar_Struct.Q_ini(1,i) );
        Coord.setSpeedValue ( osimState, TargetVar_Struct.U_ini(1,i) );
    end

    for i = 1:Num_Targets
        TargetVar_Struct.TargetBody{i,1} = Contt_Setting.TargetBody{i,1};
        TargetVar_Struct.TargetSwitch{i,1} = Contt_Setting.TargetSwitch{i,1};
        TargetVar_Struct.TargetSub_Pointer{i,1} = Contt_Setting.TargetSub_Pointer{i,1};
        TargetVar_Struct.TargetDes_Pointer{i,1} = Contt_Setting.TargetDes_Pointer{i,1};
        for j = 1:Num_Contacts;
            TargetVar_Struct.TargetContact{i,j} = char(Contt_Setting{i,j+4});
        end
        for j = 1:osimModel.getCoordinateSet.getSize
            if strcmp(osimModel.getBodySet.get(osimModel.getCoordinateSet.get(j-1).getBodyIndex).getName,TargetVar_Struct.TargetBody{i,1})
                TargetVar_Struct.Target_RelCoord{i,1} = char(osimModel.getCoordinateSet.get(j-1).getName);
                break;
            end
        end
    end
   
    Out_Struct = TargetVar_Struct;

end