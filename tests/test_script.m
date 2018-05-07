vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections

clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);

numberOfBills = 8; % zmianiam liczbe bill�w

if (clientID>-1)
    display('Connection successful');
    paramedic = getRats(numberOfBills,clientID,vrep);  % tworze 2 ratownik�w
    
    [~,saveMe]=vrep.simxGetObjectHandle(clientID,'Poszkodowany',vrep.simx_opmode_blocking);
    joints = zeros(size(paramedic,2),4); % alokuje macierz na jointy 2x4
    for i=1:size(paramedic,2)
        joints(i,:) = getJoints(i,clientID,vrep); % pobieram jointy ka�dego ratownika
    end
    
    bills_sensor = readSensors(clientID,vrep,numberOfBills); % init czujniki
    % inicjalizuje pozycje
    positions = zeros(size(paramedic,2),3);
    for i=1:size(paramedic,2)
         [~ , position] = vrep.simxGetObjectPosition(clientID,paramedic(i),-1,vrep.simx_opmode_blocking);
         positions(i,:) = position;
    end
    posOfJoints = getPosOfJoints(1); % init joints
    
    k=0;
    dx = 0.008;
    dy = -0.004;
    flag = 1;
    time = 0;
    while flag
        time = time +1;
        if (mod(time,5) == 0)
            if k ~= 21 % size of position arrays
                k = k+1;
            else
                k = 1;
            end
            posOfJoints = getPosOfJoints(k);
        end

        %zczytuje wartosci z czujnik�w i sprawdzam czy poszkodowany jest
        %znaleziony
        for i=1 : size(bills_sensor,2)
            [~,detectionState,detectionPoint,detectedObject,~]= vrep.simxReadProximitySensor(clientID,bills_sensor(i),vrep.simx_opmode_streaming);
            if(detectionState)
                name = strcat('Przedmiot wykryty: ',num2str(detectedObject));
                name = strcat(name,', W odleg�o�ci:  ');
                name = strcat(name,num2str(norm(detectionPoint)));
                name = strcat(name,',  Przez ratownika: ');
                name = strcat(name,num2str(i));
                disp(name)
                if(saveMe == detectedObject)
                    disp('Misja zako�czona sukcesem')
                    flag =0;
                end
            end
        end
          
        for i=1 : size(paramedic,2)
            for j=1:4 %zmieniam pozycje n�g
                vrep.simxSetJointPosition(clientID,joints(i,j),posOfJoints(j),vrep.simx_opmode_streaming);
            end
             % id� i obracam
            positions(i,1)=positions(i,1)+dx;
            positions(i,2)=positions(i,2)+dy;
            vrep.simxSetObjectPosition(clientID,paramedic(i),-1,[positions(i,1),positions(i,2),positions(i,3)],vrep.simx_opmode_oneshot);
            orient = setOrientationFromPosition(dx,dy);
            vrep.simxSetObjectOrientation(clientID,paramedic(i),-1,[0 0 orient],vrep.simx_opmode_oneshot);
        end
    end
        %% koniec wiec pozycja stoj�ca
        for i=1 : size(paramedic,2)
            for j=1:4
                vrep.simxSetJointPosition(clientID,joints(i,j),0,vrep.simx_opmode_oneshot);
            end
        end
        
        vrep.simxFinish(-1);
end
vrep.delete();