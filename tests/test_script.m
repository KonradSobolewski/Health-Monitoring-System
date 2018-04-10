vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections

clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);



if (clientID>-1)
    display('Connection successful');
    paramedic = getRats(2,clientID,vrep);  % tworze 2 ratowników 
    
    joints = zeros(size(paramedic,2),4); % alokuje macierz na jointy 2x4
    for i=1:size(paramedic,2)
           joints(i,:) = getJoints(i,clientID,vrep); % pobieram jointy ka¿dego ratownika
    end
    k=0;
    v = 0.05;
    flag = 1;
    angles = zeros(size(paramedic,2),3);
    positions = zeros(size(paramedic,2),3);
    while flag
        if k ~= 21 % size of position arrays
            k = k+1;
        else
            k = 1;
        end
        posOfJoints = getPosOfJoints(k);
        
        % pobieram aktualn¹ pozycje i k¹t
        for i=1:size(paramedic,2) 
             [~ , angle] = vrep.simxGetObjectOrientation(clientID,paramedic(i),-1,vrep.simx_opmode_blocking);
             [~ , position] = vrep.simxGetObjectPosition(clientID,paramedic(i),-1,vrep.simx_opmode_blocking);
             angles(i,:) = angle
             positions(i,:) = position;
        end
        %zmieniam pozycje nóg
        for i=1 : size(paramedic,2)
             for j=1:4
                vrep.simxSetJointPosition(clientID,joints(i,j),posOfJoints(j),vrep.simx_opmode_oneshot);
             end
        end
        % idê i obracam
        for i=1 : size(paramedic,2)
             vrep.simxSetObjectOrientation(clientID,paramedic(i),-1,[0 0 0],vrep.simx_opmode_oneshot); % chwilowo na  0 0 0 
             vrep.simxSetObjectPosition(clientID,paramedic(i),-1,[positions(i,1)+v,positions(i,2),positions(i,3)],vrep.simx_opmode_oneshot);
        end
    end
    %% koniec wiec pozycja stoj¹ca
    for i=1 : size(paramedic,2)
        for j=1:4
            vrep.simxSetJointPosition(clientID,joints(i,j),0,vrep.simx_opmode_oneshot);
        end
    end
    
    pause(2)
    vrep.simxFinish(-1);
end
vrep.delete();