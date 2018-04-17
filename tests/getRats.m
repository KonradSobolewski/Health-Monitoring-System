function arrayOfRat = getRats(counter,clientID,vrep)
    arrayOfRat = [];
    for i=1:counter
        name = strcat('Ratownik_',num2str(i));
        [~,rat]=vrep.simxGetObjectHandle(clientID,name,vrep.simx_opmode_blocking);
        arrayOfRat = [arrayOfRat, rat];
    end
end