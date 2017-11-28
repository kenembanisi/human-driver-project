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
    WholeVarStruct.ContactBd = ''; % container for contact body name
    WholeVarStruct.ContactFr = zeros(1,6); % variable for storing contact force-torque
    for i = 1:StateValNames.getSize
        WholeVarStruct.StateLabl{i,1} = char(StateValNames.getitem(i-1));
        SpString = split(WholeVarStruct.StateLabl{i,1},'/');
        JointName = char(SpString(1)); % add char -- 
        CoordName = char(SpString(2));
        ValueName = char(SpString(3));
        Coord = osimModel.getCoordinateSet().get(CoordName);
        if strcmp(ValueName, 'value')
            WholeVarStruct.StateVals(i,1) = Coord.getValue(osimState);        
        else
            WholeVarStruct.StateVals(i,1) = Coord.getSpeedValue(osimState);        
        end
    end

    for i = 1:CoordSet.getSize
        WholeVarStruct.CoordSet{i,1} = char(CoordSet.get(i-1).getName);
    end

    for i = 1:JointSet.getSize
        WholeVarStruct.JointSet{i,1} = char(JointSet.get(i-1).getName);
    end

    Out_Struct = WholeVarStruct;

end