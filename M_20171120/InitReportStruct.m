function Out_Struct = InitReportStruct (TargetVar_Struct)

%     % Import the OpenSim modeling classes
%     import org.opensim.modeling.*
% 
%     if str2num(char(org.opensim.modeling.opensimCommon.GetVersion())) < 4.0
%         disp(['Please install OpenSim 4.0 or greater than 4.0']);
%         exit(-1);
%     end
% 
%     pause(1);

    pathname = TargetVar_Struct.pathname;
    Sys_Name = TargetVar_Struct.SysName;
    
    ForceSets = TargetVar_Struct.Model.getForceSet();

    Reporter_Struct = struct();

    Reporter_Struct.contact_flag = false;
%     Model_Setting = readtable(strcat(pathname,Sys_Name,'_setting.txt'),'Delimiter','tab','ReadRowNames',false,'ReadVariableNames',true);
    if (exist (strcat(pathname,Sys_Name,'_reportcontact.txt')))
        RCont_Setting = readtable(strcat(pathname,Sys_Name,'_reportcontact.txt'),'Delimiter','tab','ReadRowNames',false,'ReadVariableNames',true);
        Reporter_Struct.contact_flag = true;
        Reporter_Struct.contact_report = struct();
        Reporter_Struct.contact_report.File = [Sys_Name,'-',TargetVar_Struct.bufferstamp,'_ContactForce.sto'];
        Reporter_Struct.contact_report.number = size(RCont_Setting,1);
        Reporter_Struct.contact_report.contact= cell(1,Reporter_Struct.contact_report.number);
        Reporter_Struct.contact_report.targetbody = cell(1,Reporter_Struct.contact_report.number);
        Reporter_Struct.contact_report.geometry = cell(1,Reporter_Struct.contact_report.number);
        Reporter_Struct.contact_report.existence = zeros(1,Reporter_Struct.contact_report.number);
        for i = 1:Reporter_Struct.contact_report.number
            Reporter_Struct.contact_report.contact{1,i} = RCont_Setting.Contact{i,1};
            Reporter_Struct.contact_report.targetbody{1,i} = RCont_Setting.TargetBody{i,1};
            Reporter_Struct.contact_report.geometry{1,i}= RCont_Setting.Geometry{i,1};
            for j = 1:ForceSets.getSize
                if strcmp(ForceSets.get(j-1).getName(),RCont_Setting.Contact{i,1})
                    Labels = ForceSets.get(j-1).getRecordLabels;
                    for k = 1:Labels.getSize
                        if strfind(Labels.get(k-1),RCont_Setting.TargetBody{i,1})
                            Reporter_Struct.contact_report.existence(1,i) = 1.;
                            break;
                        end
                    end
                    if Reporter_Struct.contact_report.existence(1,i)
                        break;
                    end
                end
            end
        end

        for i = Reporter_Struct.contact_report.number:-1:1
            Reporter_Struct.contact_report.existence(1,i);
            if ~Reporter_Struct.contact_report.existence(1,i)
                Reporter_Struct.contact_report.contact(:,i) = [];
                Reporter_Struct.contact_report.targetbody(:,i)=[];
                Reporter_Struct.contact_report.geometry(:,i)=[];
                Reporter_Struct.contact_report.existence(:,i)=[];
            end
        end
        Reporter_Struct.contact_report.number = length(Reporter_Struct.contact_report.contact);

        Reporter_Struct.contact_report.dataLabls = cell(Reporter_Struct.contact_report.number*8, 1);

        for i = 1:Reporter_Struct.contact_report.number
            for j = 1:ForceSets.getSize
                if strcmp(ForceSets.get(j-1).getName(),RCont_Setting.Contact{i,1})
                    Labels = ForceSets.get(j-1).getRecordLabels;
                    Count = 0;
                    for k = 1:Labels.getSize
                        if strfind(Labels.get(k-1),RCont_Setting.TargetBody{i,1})
                            Count = Count + 1;
                            Reporter_Struct.contact_report.dataLabls{(i-1)*8+Count,1} = char(Labels.get(k-1));
                        end
                        if Count >= 6
                            Count = Count + 1;
                            Reporter_Struct.contact_report.dataLabls{(i-1)*8+Count,1}...
                                = [RCont_Setting.Contact{i,1},'.',RCont_Setting.TargetBody{i,1},'.resultant_force'];
                            Count = Count + 1;
                            Reporter_Struct.contact_report.dataLabls{(i-1)*8+Count,1}...
                                = [RCont_Setting.Contact{i,1},'.',RCont_Setting.TargetBody{i,1},'.resultant_torque'];
                            break;
                        end
                    end
                    if Count
                        break;
                    end
                end
            end
        end
        
        Reporter_Struct.contact_report = rmfield(Reporter_Struct.contact_report,'existence');
        
        
    end

    Out_Struct = Reporter_Struct;

end