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
    positions(5,:)
    posOfJoints = getPosOfJoints(1); % init joints
    
    temp = ones(1,8)*36.6;
    press = ones(1,8)*130;
    puls = ones(1,8)*75;
    
    k=0;
    dx= zeros(1,8);
    dy= zeros(1,8);
    goDown = zeros(1,8);
    xMax = 62.5;
    xMin = -17;
    yMax = 38.5;
    yMin = - 40;
    v = 0.008;
    for i=1:numberOfBills
        dx(i) = v;
        dy(i) = -v;
    end
    
    dead = zeros(1,8);
    flag = 1;
    time = 0;
    while flag      
        
        bad = zeros(1,8);
        time = time +1;
        if (mod(time,5) == 0)
            if k ~= 21 % size of position arrays
                k = k+1;
            else
                k = 1;
            end
            posOfJoints = getPosOfJoints(k);
        end
        
        if(mod(time,1000)==0 )
            lucky_guy = randi(8,1);
            if(~dead(lucky_guy))
                temp(lucky_guy) = 36.6;
                press(lucky_guy) = 130;
                puls(lucky_guy) = 75;
            end
        end
        
        if (mod(time,30) == 0)
            temp = temp + rand(1,8)/4 - 0.125;  
            press = press + rand(1,8)*2 - 1;
            puls = puls + rand(1,8) - 0.5;
        end
          
        for i=1 : size(paramedic,2)
            
            if(dead(i) == 1)
                 vrep.simxSetObjectOrientation(clientID,paramedic(i),-1,[1.5 0 0],vrep.simx_opmode_oneshot);
                 vrep.simxSetObjectPosition(clientID,paramedic(i),-1,positions(i,:)+[0 0 0.15],vrep.simx_opmode_oneshot);
                 continue;
            end
            
            if( temp(i) < 35 || temp(i) > 39)
                bad(i) = bad(i) + 1;                
            end
            
            if( press(i) < 85 || press(i) > 180 )
               bad(i) = bad(i) + 1; 
            end
            
            if( puls(i) < 35 || press(i) > 130 )
               bad(i) = bad(i) + 1; 
            end
            
            if( bad(i) >= 2 )
               dead(i) = 1; 
            end


            for j=1:4 %zmieniam pozycje n�g
                vrep.simxSetJointPosition(clientID,joints(i,j),posOfJoints(j),vrep.simx_opmode_streaming);
            end
            
            if positions(i,1) > xMax - abs(dx(i)) && positions(i,1) < xMax + abs(dx(i)) && dx(i)>0 
                dx(i) = -v;
            elseif(positions(i,1) > xMin - abs(dx(i)) && positions(i,1) < xMin + abs(dx(i)) && dx(i) < 0 )
                dx(i) = v;
            end
            
            if positions(i,2) > yMax - abs(dy(i)) && positions(i,2) < yMax + abs(dy(i)) && dy(i)>0 
                dy(i) = -v;
            elseif positions(i,2) > yMin - abs(dy(i)) && positions(i,2) < yMin + abs(dy(i)) && dy(i)<0 
                dy(i) = v;
            end
            
            %zczytuje wartosci z czujnik�w i sprawdzam czy poszkodowany jest
            %znaleziony
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
            
            % id� i obracam
            positions(i,1)=positions(i,1)+dx(i);
            positions(i,2)=positions(i,2)+dy(i);
            vrep.simxSetObjectPosition(clientID,paramedic(i),-1,[positions(i,1),positions(i,2),positions(i,3)],vrep.simx_opmode_oneshot);
            orient = setOrientationFromPosition(dx(i),dy(i));
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