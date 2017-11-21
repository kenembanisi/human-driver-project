function Out_Des_Position = getCurr_DesPoint_TaskSpace (Marker_Param,current_time,Des_Trj_TaskSpace)
    
    Des_Size = round((Marker_Param.end - Marker_Param.start)*Marker_Param.freq + 1);

    desTime_adr = 1;
    
    if current_time < Marker_Param.start
        desTime_adr = 1;
    elseif current_time > Marker_Param.end
        desTime_adr = Des_Size;
    else
        for j = 1:Des_Size
            if current_time < (Marker_Param.start + (j-1)*Marker_Param.cycle) % Des_Time(1,j)
                desTime_adr = j;
                break;
            end
        end
    end
    
    if desTime_adr > Des_Size
        desTime_adr = Des_Size;
    end
    
    
    Out_Des_Position = Des_Trj_TaskSpace(desTime_adr,:);
    
    if length(Out_Des_Position)<3
        output = zeros(1,3);
        for i = 1:3
            output(1,i) = Des_Trj_TaskSpace(desTime_adr,i);
        end
        Out_Des_Position = output;
    end

end