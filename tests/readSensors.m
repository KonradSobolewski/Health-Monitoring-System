function sensors = readSensors(clientID,vrep,bills)
    sensors = [];
    for i=1:bills
        name = strcat('Bill_main_sensor',num2str(i));
        [~,Bill_sensor] = vrep.simxGetObjectHandle(clientID,name,vrep.simx_opmode_blocking);
        sensors = [sensors,Bill_sensor];
    end
end