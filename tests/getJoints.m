function joints = getJoints(i,clientID,vrep)
    ratownik = strcat('Ratownik_',num2str(i));
    fullname = strcat(ratownik,'_leftLegJoint');
    [~,handleLeftLeg]=vrep.simxGetObjectHandle(clientID,fullname,vrep.simx_opmode_blocking);
    fullname = strcat(ratownik,'_rightLegJoint');
    [~,handleRightLeg]=vrep.simxGetObjectHandle(clientID,fullname,vrep.simx_opmode_blocking);
    fullname = strcat(ratownik,'_leftKneeJoint');
    [~,handleLeftKnee]=vrep.simxGetObjectHandle(clientID,fullname,vrep.simx_opmode_blocking);
    fullname = strcat(ratownik,'_rightKneeJoint');
    [~,handleRightKnee]=vrep.simxGetObjectHandle(clientID,fullname,vrep.simx_opmode_blocking);
    
    joints = [handleLeftLeg , handleRightLeg , handleLeftKnee , handleRightKnee];
end
