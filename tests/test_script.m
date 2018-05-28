clc
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections

clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
wayOfWalking = 1;  % 1 promieniscie 2 równolegle
numberOfBills = 8; % zmianiam liczbe billów

if (clientID>-1)
    display('Connection successful');
    paramedic = getRats(numberOfBills,clientID,vrep);  % tworze 8 ratowników
    
    [~,saveMe]=vrep.simxGetObjectHandle(clientID,'Poszkodowany',vrep.simx_opmode_blocking);
    joints = zeros(size(paramedic,2),4); % alokuje macierz na jointy 2x4
    for i=1:size(paramedic,2)
        joints(i,:) = getJoints(i,clientID,vrep); % pobieram jointy ka¿dego ratownika
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
    
    temp = ones(1,numberOfBills)*36.6;
    press = ones(1,numberOfBills)*130;
    puls = ones(1,numberOfBills)*75;
    
    k=0;
    dx= zeros(1,numberOfBills);
    dy= zeros(1,numberOfBills);
    avoid = zeros(1,numberOfBills);
    beforAvoid = zeros(numberOfBills,2); %old dx old dy
    avoidStart = zeros(numberOfBills,2);
    
    signals = zeros(numberOfBills,numberOfBills); % % od kogo do kogo
    signalSpeed = 0.1;
    r = zeros(1,numberOfBills);
    
    xMax = 62.5;
    xMin = -17;
    yMax = 38.5;
    yMin = - 40;
    v = 0.008;
    for i=1:numberOfBills
        if wayOfWalking == 1
            [dx(i),dy(i)] = setPositionFromOrientation(0.3 - 0.2*i,sqrt((v^2)*2));
        else
            dx(i) = v;
            dy(i) = -v;
        end
    end
    
    dead = zeros(1,numberOfBills);
    stay = zeros(1,numberOfBills);
    flag = 1;
    time = 0;
    inRange = zeros(1,numberOfBills);
    savers = zeros(1,numberOfBills); %kto kogo ratuje
   
    while flag      
        bad = zeros(1,numberOfBills);
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
            lucky_guy = randi(numberOfBills,1);
            if(~dead(lucky_guy))
                temp(lucky_guy) = 36.6;
                press(lucky_guy) = 130;
                puls(lucky_guy) = 75;
            end
        end
        
        if(sum(dead) == numberOfBills-1)
            flag =0;
            disp('Mission failed')
        end
        
        if (mod(time,30) == 0)
            temp = temp + rand(1,numberOfBills)/4 - 0.125;  
            press = press + rand(1,numberOfBills)*2 - 1;
            puls = puls + rand(1,numberOfBills) - 0.5;
        end
          
        for i=1 : numberOfBills            
            if(dead(i) == 1)
                vrep.simxSetObjectOrientation(clientID,paramedic(i),-1,[1.5 0 0],vrep.simx_opmode_oneshot);
                r(i) = r(i) + signalSpeed;
                d = sqrt((positions(1,1) - positions(i,1))^2 + (positions(1,2) - positions(i,2))^2);
                if( d < r(i) && inRange(i) == 0 )
                    inRange(i) = 1;
                    pause(3);
                    savers = [0 0 0 i i 0 0 0];  %%  TODO
                end
                continue;
            end
            
            if(stay(i) == 1 )
               continue;
            else
               for j=1:4 %zmieniam pozycje nóg
                    vrep.simxSetJointPosition(clientID,joints(i,j),posOfJoints(j),vrep.simx_opmode_streaming);
               end
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
            
            if( bad(i) >= 2 && i~=1 && sum(dead)<3) 
               dead(i) = 1; 
            end
            if( bad(i) >= 2 && i~=1 && sum(dead)>=3) 
               temp(i) = 36.6;
               press(i) = 130;
               puls(i) = 75;
            end
                        
            %zczytuje wartosci z czujników i sprawdzam czy poszkodowany jest
            %znaleziony , decycja jak omijam
            [~,detectionState,detectionPoint,detectedObject,~]= vrep.simxReadProximitySensor(clientID,bills_sensor(i),vrep.simx_opmode_streaming);
            if(detectionState)
%                 name = strcat('Przedmiot wykryty: ',num2str(detectedObject));
%                 name = strcat(name,', W odleg³oœci:  ');
%                 name = strcat(name,num2str(norm(detectionPoint)));
%                 name = strcat(name,',  Przez ratownika: ');
%                 name = strcat(name,num2str(i));
%                 disp(name) 
                if( detectionPoint(1) < 0.5 && detectionPoint(1) > 0 && detectionPoint(3) < 1.5 && avoid(i)==0) % skrêt w prawo
                    avoid(i) = 1;
                    beforAvoid(i,1) = dx(i);
                    beforAvoid(i,2) = dy(i);
                    avoidStart(i) = time;
                elseif ( detectionPoint(1) > -0.5 && detectionPoint(1) < 0 && detectionPoint(3) < 1.5 && avoid(i)==0) % skrêt w lewo
                    avoid(i) = 2;
                    beforAvoid(i,1) = dx(i);
                    beforAvoid(i,2) = dy(i);
                    avoidStart(i) = time;
                end
                if(saveMe == detectedObject)
                    disp('Misja zakoñczona sukcesem')
                    flag =0;
                end
            end
            % chodzenie w kierunku ratownika chorego
            if( savers(i) ~= 0 )
                if( abs(positions(i,1) - positions(savers(i),1)) < 1 && abs(positions(i,2) - positions(savers(i),2)) < 1)
                    stay(i) = 1;
                end
                orient2 = setOrientationFromPosition(positions(savers(i),1) - positions(i,1), positions(savers(i),2)- positions(i,2));
                [dx(i),dy(i)] = setPositionFromOrientation(orient2,sqrt((v^2)*2));
            end
            
            %predykcja nastêpnego po³o¿enia w normalnym przypadku
            if avoid(i) == 0
                if(savers(i) == 0 )
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
                end
            else    
                [dx(i),dy(i),avoid(i)] = avoiding(beforAvoid(i,1),beforAvoid(i,2),avoidStart(i),time,avoid(i));
            end
            
%             idê i obracam
            positions(i,1)=positions(i,1)+dx(i);
            positions(i,2)=positions(i,2)+dy(i);
            vrep.simxSetObjectPosition(clientID,paramedic(i),-1,[positions(i,1),positions(i,2),positions(i,3)],vrep.simx_opmode_oneshot);
            orient = setOrientationFromPosition(dx(i),dy(i));
            vrep.simxSetObjectOrientation(clientID,paramedic(i),-1,[0 0 orient],vrep.simx_opmode_oneshot);
        end
    end
        %% koniec wiec pozycja stoj¹ca
        for i=1 : size(paramedic,2)
            for j=1:4
                vrep.simxSetJointPosition(clientID,joints(i,j),0,vrep.simx_opmode_oneshot);
            end
        end
        
        vrep.simxFinish(-1);
end
vrep.delete();