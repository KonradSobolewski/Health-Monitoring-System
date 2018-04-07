vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections

clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

if (clientID>-1)
    disp('Connection successful');
    
    [returnCode,Bill]=vrep.simxGetObjectHandle(clientID,'Bill',vrep.simx_opmode_blocking);
    for i=1:100
        [returnCode]=vrep.simxSetObjectPosition(clientID,Bill,-1,[-2+i*0.04,0,0],vrep.simx_opmode_blocking);
        pause(0.03);
    end
    [returnCode,Bill]=vrep.simxCallScriptFunction(clientID,'Bill#0',6,'sysCall_actuation',[],[],[],[],vrep.simx_opmode_blocking)
    pause(5)
    
    vrep.simxFinish(-1);
    
end

vrep.delete();