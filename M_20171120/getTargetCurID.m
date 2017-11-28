function Out = getTargetCurID(TargetVar_Struct)
    Out = 0;
    for i = 1:length(TargetVar_Struct.TargetBody)
        if strcmp(TargetVar_Struct.TargetBody{i},TargetVar_Struct.Target_current)
            Out = i;
            return;
        end
    end
end