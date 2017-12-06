function  Output = getPedalData (fullfileName)

    % Import the OpenSim modeling classes
    import org.opensim.modeling.*

    % Get trc data to determine time range
    PedalData = Storage(fullfileName);

    Target_Time = ArrayDouble();
    Pedal_Position  = ArrayDouble();
    Num_Target = PedalData.getTimeColumn(Target_Time);
    Pedal_Name = PedalData.getColumnLabels().get(1);
    PedalData.getDataColumn(Pedal_Name,Pedal_Position);

    PedalData_array = zeros(2,Num_Target);
    for i = 1:Num_Target
        PedalData_array(1,i) = Target_Time.get(i-1);
        PedalData_array(2,i) = Pedal_Position.get(i-1);
    end

    Output = PedalData_array;

end