function Out_Struct = InitWholeVarStruct (osimModel, osimState)

    % Import the OpenSim modeling classes
    import org.opensim.modeling.*


    JointSet = osimModel.getJointSet();
    CoordSet = osimModel.getCoordinateSet();
    StateValNames      = osimModel.getStateVariableNames();

    WholeVarStruct = struct();
    WholeVarStruct.JointSet  = cell(JointSet.getSize,1);
    WholeVarStruct.CoordSet  = cell(CoordSet.getSize,1);
    WholeVarStruct.StateLabl = cell(StateValNames.getSize,1);
    WholeVarStruct.StateVals = zeros(StateValNames.getSize,1);
    WholeVarStruct.ContactBd = '';
    WholeVarStruct.ContactFr = zeros(1,6);
    for i = 1:StateValNames.getSize
        WholeVarStruct.StateLabl{i,1} = StateValNames.getitem(i-1);
        SpString = split(WholeVarStruct.StateLabl{i,1},'/');
        JointName = SpString(1);
        CoordName = SpString(2);
        ValueName = SpString(3);
        Coord = osimModel.getCoordinateSet().get(CoordName);
        if strcmp(ValueName, 'value')
            WholeVarStruct.StateVals(i,1) = Coord.getValue(osimState);        
        else
            WholeVarStruct.StateVals(i,1) = Coord.getSpeedValue(osimState);        
        end
    end

    for i = 1:CoordSet.getSize
        WholeVarStruct.CoordSet{i,1} = CoordSet.get(i-1);
    end

    for i = 1:JointSet.getSize
        WholeVarStruct.JointSet{i,1} = JointSet.get(i-1);
    end

    Out_Struct = WholeVarStruct;

end