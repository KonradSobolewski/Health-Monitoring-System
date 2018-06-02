clc
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections

clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
wayOfWalking = 1;  % 1 promieniscie 2 równolegle
numberOfBills = 8; % zmianiam liczbe billów

if (clientID>-1)
    display('Connection successful');
    paramedic = getRats(numberOfBills,clientID,vrep);  % tworze 8 ratowników
    hierarchy = randperm(length(paramedic));
    leader = find(hierarchy==1);
    
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
    sys_press = ones(1,numberOfBills)*120;
    dias_press = ones(1,numberOfBills)*80;
    puls = ones(1,numberOfBills)*75;
    prev_sys_press = ones(6000,numberOfBills)*120;
    prev_dias_press = ones(6000,numberOfBills)*80;
    prev_puls = ones(6000,numberOfBills)*75;
    
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
            [dx(i),dy(i)] = setPositionFromOrientation(0.1 - 0.2*i,sqrt((v^2)*2));
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
    saviors = zeros(1,numberOfBills); %kto kogo ratuje
 
 % pêtla while - pêtla w³aœciwa symulacji
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
        
        % raz na 1000 iteracji jeden losowy ratownik cudownie zdrowieje
        if(mod(time,1000)==0 )
            lucky_guy = randi(numberOfBills,1);
            if(~dead(lucky_guy))
                temp(lucky_guy) = 36.6;
                sys_press(lucky_guy) = 120;
                dias_press(lucky_guy) = 80;
                puls(lucky_guy) = 75;
            end
        end
        
        % to w³aœciwie mo¿na wywaliæ, bo zak³adamy, ¿e maks dwóch potrzebuje pomocy
        if(sum(dead) == numberOfBills-1)
            flag =0;
            disp('Mission failed')
        end
        
        % losowe zmienianie wartoœci odczytów ratowników
        if (mod(time,30) == 0)
            temp = temp + rand(1,numberOfBills)/4 - 0.125;
            sys_press = sys_press + rand(1,numberOfBills)*2 - 1;
            prev_sys_press = [sys_press;prev_sys_press(1:(end-1),:)];
            dias_press = dias_press + rand(1,numberOfBills)*1.3 - 0.6;
            prev_dias_press = [dias_press;prev_dias_press(1:(end-1),:)];
            puls = puls + rand(1,numberOfBills) - 0.5;
            prev_puls = [puls;prev_puls(1:(end-1),:)];
        end
        
        % w tej pêtli for iterujemy po wszystkich ratownikach
        for i=1 : numberOfBills 
            % jeœli dany ratownik jest kwalifikuje siê do wys³ania sygna³u SOS to k³adziemy go
            if(dead(i) == 1)
                vrep.simxSetObjectOrientation(clientID,paramedic(i),-1,[1.5 0 0],vrep.simx_opmode_oneshot);
                % wyslanie sygna³u sos
                r(i) = r(i) + signalSpeed;
                d = sqrt((positions(1,1) - positions(i,1))^2 + (positions(1,2) - positions(i,2))^2);
                if( d < r(i) && inRange(i) == 0 )
                    inRange(i) = 1;
                    pause(3);
                    saviors = [0 0 0 i i 0 0 0];  %%  tutaj powinni byæ okreœlani ratownicy, którzy szukaj¹ kolegi
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
            
            % tutaj zaczyna siê sprawdzanie odczytów i determinowanie czy wys³aæ sygna³ sos
            bad(i) = sosSignalGenerator(temp(i),puls(i),sys_press(i),dias_press(:,i),prev_puls(:,i),prev_sys_press(:,i),prev_dias_press(:,i));
            
            if( bad(i) >= 2 && i~=1 && sum(dead)<3) 
               dead(i) = 1;
               if i==leader
                   hierarchy = hierarchy-1;
                   leader = find(hierarchy==1);
               end
            end
            if( bad(i) >= 2 && i~=1 && sum(dead)>=3) 
               temp(i) = 36.6;
               sys_press(i) = 120;
               dias_press(i) = 80;
               puls(i) = 75;
            end
            % a tu koñczy 
                        
            %sczytuje wartosci z czujników i sprawdzam czy poszkodowany jest znaleziony , decycja jak omijam
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
            if( saviors(i) ~= 0 )
                if( abs(positions(i,1) - positions(saviors(i),1)) < 1 && abs(positions(i,2) - positions(saviors(i),2)) < 1)
                    stay(i) = 1;
                end
                orient2 = setOrientationFromPosition(positions(saviors(i),1) - positions(i,1), positions(saviors(i),2)- positions(i,2));
                [dx(i),dy(i)] = setPositionFromOrientation(orient2,sqrt((v^2)*2));
            end
            
            %predykcja nastêpnego po³o¿enia w normalnym przypadku
            if avoid(i) == 0
                if(saviors(i) == 0 )
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