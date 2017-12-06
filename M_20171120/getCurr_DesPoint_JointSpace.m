function [Out_Des_Q,Out_Des_U] = getCurr_DesPoint_JointSpace (Marker_Param,current_time,TargetVar_Struct,WholeVarStruct,Des_Trj_JntSpace,Des_Trj_JntLabl)
    
    Des_Size = round((Marker_Param.end - Marker_Param.start)*Marker_Param.freq + 1);

    if current_time < Marker_Param.start
        desTime_adr = 1;
    elseif current_time > Marker_Param.end
        desTime_adr = Des_Size;
    else
        desTime_adr = Des_Size;
        for j = 1:Des_Size
            if current_time < (Marker_Param.start + (j-1)*Marker_Param.cycle) % Des_Time(1,j)
                desTime_adr = j;
                break;
            end
        end
    end

    Des_Q = zeros(1,TargetVar_Struct.Number);
    Des_U = zeros(1,TargetVar_Struct.Number);
    
    for j = 1:TargetVar_Struct.Number
        for k = 1:size(WholeVarStruct.CoordSet,1)
            if strcmp(TargetVar_Struct.Cent_cor_name{1,j},Des_Trj_JntLabl{k,1})
                Des_Q(1,j) = Des_Trj_JntSpace(k,desTime_adr)/180*pi;
                if desTime_adr > 1
                    Des_U(1,j) = (Des_Trj_JntSpace(k,desTime_adr)-...
                        Des_Trj_JntSpace(k,desTime_adr-1))/180.*pi/...
                        ( (Marker_Param.start + (desTime_adr-1)*Marker_Param.cycle)...
                        - (Marker_Param.start + ((desTime_adr-1)-1)*Marker_Param.cycle) );
                else
                    Des_U(1,j) = 0.;
                end
                break;
            end
        end
    end

    Out_Des_Q = Des_Q;
    Out_Des_U = Des_U;
end