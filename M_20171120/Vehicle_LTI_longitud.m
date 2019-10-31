function Output = Vehicle_LTI_longitud(Input_Struct)

    % Read preceding vehicle velocity
    Preced_vel_sim	= importdata('Preced_vel.csv');
    
    Preced_vel_sim	= Input_Struct.Preced_vel;
    T_hwm           = Input_Struct.T_hwm;
    Vm0             = Input_Struct.Vm0;
    StartTime       = Input_Struct.Start_Time;
    TermTime        = Input_Struct.End_Time; 
    TimeStep        = Input_Struct.TimeStep; 
    Respld_Pgm2     = Input_Struct.G_Pedal_pos;
    Respld_Pbm2     = Input_Struct.B_Pedal_pos;

    % constant parameter for vehicle and driver models
    T_hwm=1.5; 
    Vm0=72.1873; 
    
    Cair=0.011; dlong=0; HA=0; HR=2; HV=6; Kg=0.028; 
    Tg=0.0081; Tp=1; Tb=0.01; Kb=Kg*3.;
    Rm0=Vm0/3.6*T_hwm;

    % Simulation time
    StartTime=0.00; TimeStep=1/20; TermTime=100.0; 
    StopTime=StartTime+TimeStep;
    NumSteps=(TermTime-StartTime)/TimeStep;

    %% Data space in whole simulation time
    % Preceding vehicle velocity [m]
    Respld_PrecVel = zeros(NumSteps+1,2);
    % Position of Gas pedal [%]
    Respld_Pgm2 = zeros(NumSteps+1,2);
    % Position of Brake pedal [%]
    Respld_Pbm2 = zeros(NumSteps+1,2);
    % Acceleration of host vehicle [m/(s^2)]
    Respld_Acc_Veh2 = zeros(NumSteps+1,2);
    % Desired pedal position [%]
    Respld_Pdls_d2 = zeros(NumSteps+1,2);

    % Outputs_accel_frm_brk = zeros(NumSteps+1,1+8);
    Outputs_va_g2      = zeros(NumSteps+1,1+1);
    Outputs_va_b2      = zeros(NumSteps+1,1+1);
    Outputs_vb2        = zeros(NumSteps+1,1+7);
    Outputs_pedal_fct2 = zeros(NumSteps+1,1+1);

    %% Initialization
    % t=1
    Respld_PrecVel(1,1) = interp1(Preced_vel_sim(:,1),Preced_vel_sim(:,1),StartTime);
    Respld_PrecVel(1,2) = (interp1(Preced_vel_sim(:,1),Preced_vel_sim(:,2),StartTime));
    Respld_Pgm2(1,1) = Respld_PrecVel(1,1);
    Respld_Pgm2(1,2) = 0.0;
    Respld_Pbm2(1,1) = Respld_PrecVel(1,1);
    Respld_Pbm2(1,2) = 0.0;
    Respld_Acc_Veh2(1,1) = Respld_PrecVel(1,1);
    Respld_Acc_Veh2(1,2) = 0.0;
    Respld_Pdls_d2(1,1) = Respld_PrecVel(1,1);
    Respld_Pdls_d2(1,2) = 0.0;

    % t=2
    Respld_Pdls_d2(2,1) = Respld_Pgm2(2,1);
    Respld_Pdls_d2(2,2) = 0.0;

    % Outputs t=1
    Outputs_va_g2(1,1)       = Respld_PrecVel(1,1);
    Outputs_va_b2(1,1)       = Respld_PrecVel(1,1);
    Outputs_vb2(1,1)         = Respld_PrecVel(1,1);
    Outputs_pedal_fct2(1,1)  = Respld_PrecVel(1,1);

    %% Linear model

    % System "a" from brake_p to acceleration
    Sys_tf_va_b = tf([Kb],[Tb 1]);
    Sys_ssva_b2 = ss(Sys_tf_va_b);
    Sys_Dva_b2=c2d(Sys_ssva_b2,TimeStep);

    % System "a" from gas_p to acceleration
    Sys_tf_va_g = tf([Kg],[Tg 1]);
    Sys_ssva_g2 = ss(Sys_tf_va_g);
    Sys_Dva_g2=c2d(Sys_ssva_g2,TimeStep);

    % System "b" from acceleration to pedal_desire
    Sys_tf_vb1 = tf([1],[1 Cair]); % get host vehicle's velocity
    Sys_ssvb1 = ss(Sys_tf_vb1);
    Sys_Dvb1=c2d(Sys_ssvb1,TimeStep);

%     Sys_tf_vb2 = tf([1 0],[1 Cair]); % get host vehicle's acceleration
%     Sys_ssvb2 = ss(Sys_tf_vb2);
%     Sys_Dvb2=c2d(Sys_ssvb2,TimeStep);
% 
%     Sys_tf_vb3 = tf([1],[1 Cair 0]); % get host vehicle's Milleage
%     Sys_ssvb3 = ss(Sys_tf_vb3);
%     Sys_Dvb3=c2d(Sys_ssvb3,TimeStep);
% 
%     Sys_tf_vb4 = tf([1],[1 Cair 0]); % get host vehicle's Milleage
%     Sys_ssvb4 = ss(Sys_tf_vb4);
%     Sys_Dvb4=c2d(Sys_ssvb4,TimeStep);
% 
%     Sys_tf_vb5 = tf([-HV-HR*T_hwm],[1 Cair]); % get desired pedal position
%     Sys_ssvb5 = ss(Sys_tf_vb5);
%     Sys_Dvb5=c2d(Sys_ssvb5,TimeStep);
% 
%     % System "c" from preced_vel to pedal_desire
%     Sys_tf_vc = tf([HV HR],[1 0]);
%     Sys_ssvc2 = ss(Sys_tf_vc);
%     Sys_Dvc2=c2d(Sys_ssvc2,TimeStep);

    % System "d" from pedal_desire to pedal_fact
    Sys_tf_vd = tf([1],[Tp 1]);
    Sys_ssvd2 = ss(Sys_tf_vd);
    Sys_Dvd2=c2d(Sys_ssvd2,TimeStep);




    %% Sequencial computation
    for i = 1:NumSteps
    % Initializing times
      % for Temporal variables
        Respld_PrecVel(i+1,1)    = interp1(Preced_vel_sim(:,1),Preced_vel_sim(:,1),StopTime);
        Respld_Pdls_d2(i+1,1)     = Respld_PrecVel(i+1,1);
        Outputs_pedal_fct2(i+1,1) = Respld_PrecVel(i+1,1);
        Respld_Pgm2(i+1,1)        = Respld_PrecVel(i+1,1);
        Respld_Pbm2(i+1,1)        = Respld_PrecVel(i+1,1);
        Respld_Acc_Veh2(i+1,1)    = Respld_PrecVel(i+1,1);

      % for Output variables
        Outputs_va_g2(i+1,1)      = Respld_PrecVel(i+1,1);
        Outputs_va_b2(i+1,1)      = Respld_PrecVel(i+1,1);
        Outputs_vb2(i+1,1)        = Respld_PrecVel(i+1,1);

        Outputs_vb2(1,1+2) = -1*Vm0/3.6*Cair;
        Outputs_vb2(1,1+6) = 0.0;
        Outputs_vb2(1,1+4) = Rm0;

    % Human response (time delay model) [desired pedal position >> actual pedal positions]
        if i>1
            Outputs_pedal_fct2(2:i+1,2) = lsim(Sys_Dvd2, Respld_Pdls_d2(1:i,2),Respld_Pdls_d2(1:i,1),0);
        else
            Outputs_pedal_fct2(1:i+1,2) = lsim(Sys_Dvd2, [Respld_Pdls_d2(1,2) Respld_Pdls_d2(1,2)],Respld_Pdls_d2(1:i+1,1),0);
        end

        if Outputs_pedal_fct2(i+1,2) < 0     % in case of braking
            Respld_Pgm2(i+1,2) = 0.0;
            Respld_Pbm2(i+1,2) = Outputs_pedal_fct2(i+1,2);
        else                                % in case of accelerating
            Respld_Pgm2(i+1,2) = Outputs_pedal_fct2(i+1,2);
            Respld_Pbm2(i+1,2) = 0.0;
        end

    % Determine acceleration due to Gas pedal
        Outputs_va_g2(1:i+1,2) = lsim(Sys_Dva_g2, Respld_Pgm2(1:i+1,2),Respld_Pgm2(1:i+1,1),[0]);

    % Determine acceleration due to Brake pedal
        Outputs_va_b2(1:i+1,2) = lsim(Sys_Dva_b2, Respld_Pbm2(1:i+1,2),Respld_Pbm2(1:i+1,1),[0]);

    % Sum accelerations from Gas and Brake pedals
        Respld_Acc_Veh2(1:i+1,2) = Outputs_va_g2(1:i+1,2)+Outputs_va_b2(1:i+1,2);
        %Respld_Acc_Veh(1:i+1,2) = AccelByGpedal + AccelByBrake;

    % Determine desired pedal position due to host vehicle response
        Outputs_vb2(1:i+1,1+1) = 3.6*lsim(Sys_Dvb1, Respld_Acc_Veh2(1:i+1,2),Respld_Acc_Veh2(1:i+1,1),Vm0/3.6); % host vehicle velocity [km/h] with init vel

    %     Outputs_vb2(1:i+1,1+2) = 3.6*lsim(Sys_Dvb2, Respld_Acc_Veh2(1:i+1,2),Respld_Acc_Veh2(1:i+1,1),Vm0/3.6*Cair); % host vehicle velocity [km/h] with init vel

        Outputs_vb2(i+1,1+2) = (Outputs_vb2(i+1,1+1)-Outputs_vb2(i,1+1))/(Outputs_vb2(i+1,1)-Outputs_vb2(i,1))/3.6; % host vehicle acceleration [m/s^2]
        Outputs_vb2(i+1,1+6) = (Outputs_vb2(i+1,1+1)-Outputs_vb2(1,1+1))/3.6; % host vehicle velocity [m/s]
        Outputs_vb2(i+1,1+3) = (Outputs_vb2(i+1,1+6)+Outputs_vb2(i,1+6))*(Outputs_vb2(i+1,1+0)-Outputs_vb2(i,1+0))/2.+Outputs_vb2(i,1+3); % host vehicle Milleage [m]

    % Obtain preceding vehicle velocity
        Respld_PrecVel(i+1,2) = (interp1(Preced_vel_sim(:,1),Preced_vel_sim(:,2),StopTime));
        Outputs_vb2(i+1,1+4) = ...
            ((Respld_PrecVel(i+1,2)-Outputs_vb2(i+1,1+1))/3.6...
            +(Respld_PrecVel(i,2)-Outputs_vb2(i,1+1))/3.6)...
            *(Outputs_vb2(i+1,1+0)-Outputs_vb2(i,1+0))/2.+Outputs_vb2(i,1+4);

        Outputs_vb2(i+1,1+5) = HR*(Outputs_vb2(i+1,1+4)-T_hwm*Outputs_vb2(i+1,1+1)/3.6)+HV*(Respld_PrecVel(i+1,2)-Outputs_vb2(i+1,1+1))/3.6;


    % Desired pedal position for next step
        Respld_Pdls_d2(i+1,2) = Outputs_vb2(i+1,1+5);

    % Update time information for next step    
        StartTime=StopTime;StopTime=StopTime+TimeStep;
    end
    
    
    Output = Outputs_vb2;

end
