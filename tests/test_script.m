vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections

clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

numberOfBills = 2; % zmianiam liczbe billów

if (clientID>-1)
    display('Connection successful');
    paramedic = getRats(numberOfBills,clientID,vrep);  % tworze 2 ratowników
    
    [~,saveMe]=vrep.simxGetObjectHandle(clientID,'Poszkodowany',vrep.simx_opmode_blocking);
    joints = zeros(size(paramedic,2),4); % alokuje macierz na jointy 2x4
    for i=1:size(paramedic,2)
        joints(i,:) = getJoints(i,clientID,vrep); % pobieram jointy ka¿dego ratownika
    end
    
    bills_sensor = readSensors(clientID,vrep,numberOfBills); % init czujniki
    
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
            angles(i,:) = angle;
            positions(i,:) = position;
        end
        %zmieniam pozycje nóg
        for i=1 : size(paramedic,2)
            for j=1:4
                vrep.simxSetJointPosition(clientID,joints(i,j),posOfJoints(j),vrep.simx_opmode_oneshot);
            end
        end
        
        %zczytuje wartosci z czujników i sprawdzam czy poszkodowany jest
        %znaleziony
        for i=1 : size(bills_sensor,2)
            [~,detectionState,detectionPoint,detectedObject,~]= vrep.simxReadProximitySensor(clientID,bills_sensor(i),vrep.simx_opmode_streaming);
            if(detectionState)
                disp(norm(detectionPoint));
                disp(detectedObject);
                if(saveMe == detectedObject)
                    disp('Misja zakoñczona sukcesem')
                    flag =0;
                end
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
        
        pause(3)
        dx = 1;
        dy = 1;
        vrep.simxSetObjectPosition(clientID,paramedic(1),-1,[positions(1,1)+dx,positions(1,2)+dy,positions(1,3)],vrep.simx_opmode_oneshot);
        orient = setOrientationFromPosition(dx,dy);
        vrep.simxSetObjectOrientation(clientID,paramedic(1),-1,[0 0 orient],vrep.simx_opmode_oneshot);
        pause(2)
        vrep.simxSetObjectPosition(clientID,paramedic(1),-1,[positions(1,1),positions(1,2),positions(1,3)],vrep.simx_opmode_oneshot);
        vrep.simxSetObjectOrientation(clientID,paramedic(1),-1,[0 0 0],vrep.simx_opmode_oneshot);
        pause(3)
        dx = 1;
        dy = -1;
        vrep.simxSetObjectPosition(clientID,paramedic(1),-1,[positions(1,1)+dx,positions(1,2)+dy,positions(1,3)],vrep.simx_opmode_oneshot);
        orient = setOrientationFromPosition(dx,dy);
        vrep.simxSetObjectOrientation(clientID,paramedic(1),-1,[0 0 orient],vrep.simx_opmode_oneshot);
        pause(2)
        vrep.simxSetObjectPosition(clientID,paramedic(1),-1,[positions(1,1),positions(1,2),positions(1,3)],vrep.simx_opmode_oneshot);
        vrep.simxSetObjectOrientation(clientID,paramedic(1),-1,[0 0 0],vrep.simx_opmode_oneshot);
        pause(3)
        dx = -1;
        dy = -1;
        vrep.simxSetObjectPosition(clientID,paramedic(1),-1,[positions(1,1)+dx,positions(1,2)+dy,positions(1,3)],vrep.simx_opmode_oneshot);
        orient = setOrientationFromPosition(dx,dy);
        vrep.simxSetObjectOrientation(clientID,paramedic(1),-1,[0 0 orient],vrep.simx_opmode_oneshot);
        pause(2)
        vrep.simxSetObjectPosition(clientID,paramedic(1),-1,[positions(1,1),positions(1,2),positions(1,3)],vrep.simx_opmode_oneshot);
        vrep.simxSetObjectOrientation(clientID,paramedic(1),-1,[0 0 0],vrep.simx_opmode_oneshot);
        pause(3)
        dx = -1;
        dy = 1;
        vrep.simxSetObjectPosition(clientID,paramedic(1),-1,[positions(1,1)+dx,positions(1,2)+dy,positions(1,3)],vrep.simx_opmode_oneshot);
        orient = setOrientationFromPosition(dx,dy);
        vrep.simxSetObjectOrientation(clientID,paramedic(1),-1,[0 0 orient],vrep.simx_opmode_oneshot);
        pause(2)
        vrep.simxSetObjectPosition(clientID,paramedic(1),-1,[positions(1,1),positions(1,2),positions(1,3)],vrep.simx_opmode_oneshot);
        vrep.simxSetObjectOrientation(clientID,paramedic(1),-1,[0 0 0],vrep.simx_opmode_oneshot);
        pause(3)
        
        vrep.simxFinish(-1);
    end
    vrep.delete();