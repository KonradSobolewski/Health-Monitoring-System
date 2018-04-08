vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections

clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

leftLegWaypoints=[0.237,0.228,0.175,-0.014,-0.133,-0.248,-0.323,-0.450,-0.450,...
    -0.442,-0.407,-0.410,-0.377,-0.303,-0.178,-0.111,-0.010,0.046,0.104,0.145,0.188];
rightLegWaypoints=[-0.442,-0.407,-0.410,-0.377,-0.303,-0.178,-0.111,-0.010,0.046,0.104,...
    0.145,0.188,0.237,0.228,0.175,-0.014,-0.133,-0.248,-0.323,-0.450,-0.450];

leftKneeWaypoints=[0.282,0.403,0.577,0.929,1.026,1.047,0.939,0.664,0.440,0.243,...
    0.230,0.320,0.366,0.332,0.269,0.222,0.133,0.089,0.065,0.073,0.092];
rightKneeWaypoints=[0.230,0.320,0.366,0.332,0.269,0.222,0.133,0.089,0.065,0.073,0.092,...
    0.282,0.403,0.577,0.929,1.026,1.047,0.939,0.664,0.440,0.243];


if (clientID>-1)
    display('Connection successful');
    [returnCode,Bill]=vrep.simxGetObjectHandle(clientID,'Bill',vrep.simx_opmode_blocking);
    [~,handleLeftLeg]=vrep.simxGetObjectHandle(clientID,'Bill_leftLegJoint',vrep.simx_opmode_blocking);
    [~,handleRightLeg]=vrep.simxGetObjectHandle(clientID,'Bill_rightLegJoint',vrep.simx_opmode_blocking);
    [~,handleLeftKnee]=vrep.simxGetObjectHandle(clientID,'Bill_leftKneeJoint',vrep.simx_opmode_blocking);
    [~,handleRightKnee]=vrep.simxGetObjectHandle(clientID,'Bill_rightKneeJoint',vrep.simx_opmode_blocking);
    k=0;
    v = 0.1;
    for i=1:200
        k=k+1;
        if(k == size(leftLegWaypoints,2))
            k = 1;
        end
           
        leftLeg = leftLegWaypoints(k);
        rightLeg = rightLegWaypoints(k); 
        leftKnee = leftKneeWaypoints(k);
        rightKnee = rightKneeWaypoints(k);
        [~ , position] = vrep.simxGetObjectPosition(clientID,Bill,-1,vrep.simx_opmode_blocking);
        [~ , angles] = vrep.simxGetObjectOrientation(clientID,Bill,-1,vrep.simx_opmode_blocking);
        if(  position(1) > 1  )
                v = -0.1;
                angles(3)= -3.2;  % to 180 stopni lol
        elseif ( position(1) < -2)
                v = 0.1;
                angles(3) = 0;
        end
        vrep.simxSetObjectOrientation(clientID,Bill,-1,angles,vrep.simx_opmode_oneshot);
        vrep.simxSetObjectPosition(clientID,Bill,-1,[position(1)+v,position(2),position(3)],vrep.simx_opmode_oneshot);
        vrep.simxSetJointPosition(clientID,handleLeftLeg,leftLeg...
            ,vrep.simx_opmode_oneshot);
        vrep.simxSetJointPosition(clientID,handleRightLeg,rightLeg...
            ,vrep.simx_opmode_oneshot);
            
        vrep.simxSetJointPosition(clientID,handleLeftKnee,leftKnee...
            ,vrep.simx_opmode_oneshot);
        vrep.simxSetJointPosition(clientID,handleRightKnee,rightKnee...
            ,vrep.simx_opmode_oneshot);
        
        pause(0.01)

    end
    %[returnCode,Bill]=vrep.simxCallScriptFunction(clientID,'Bill#0',6,'sysCall_actuation',[],[],[],[],vrep.simx_opmode_blocking)
    pause(5)
    
    vrep.simxFinish(-1);
    
end

vrep.delete();