%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Projekt ³¹czony z MISK i SST                                     %
%  Symulator akcji ratowniczej z wykorzystaniem œrodowiska V-rep    %
%  Jakub Pankiewicz, Konrad Sobolewski, Kinga Staszkiewicz          %
%  Warszawa 2018                                                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
close all;
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections

clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
wayOfWalking = 1;  % 1-promieniœcie 2-równolegle
fullNet = 1; % 1-pe³na siatka, 0-niepe³na siatka
numberOfBills = 8; % zmianiam liczbe Billów

clc
if (clientID>-1)
    disp('<strong>Po³¹czenie ze œrodowiskiem V-rep udane.</strong>');
    disp('Trwa inicjalizacja systemu...');
    paramedic = getRats(numberOfBills,clientID,vrep);   % tworzê 8 ratowników
    hierarchy = randperm(length(paramedic));            % hierarchia wœród ratowników
    leader = find(hierarchy==1);                        % wybór lidera
    
    % Poszkodowany - poszukiwany przez ratowników
    [~,saveMe]=vrep.simxGetObjectHandle(clientID,'Poszkodowany',vrep.simx_opmode_blocking);
    [~,saveMePosition] = vrep.simxGetObjectPosition(clientID,saveMe,-1,vrep.simx_opmode_blocking);
    vrep.simxSetObjectPosition(clientID,saveMe,-1,[saveMePosition(1:2)*(0.2*rand(1)+0.9),saveMePosition(3)],vrep.simx_opmode_oneshot);
    
    % Jointy ratowników - w celu poruszania koñczynami
    joints = zeros(size(paramedic,2),4); % alokuje macierz na jointy 2x4
    for i=1:size(paramedic,2)
        joints(i,:) = getJoints(i,clientID,vrep); % pobieram jointy ka¿dego ratownika
    end
    
    % Inicjalizacja pola widzenia - sensora w V-rep
    bills_sensor = readSensors(clientID,vrep,numberOfBills);
    
    % Inicjalizacja pozycyji ratowników
    positions = zeros(size(paramedic,2),3);
    for i=1:size(paramedic,2)
        [~ , position] = vrep.simxGetObjectPosition(clientID,paramedic(i),-1,vrep.simx_opmode_blocking);
        positions(i,:) = position;
    end
    
    % Inicjalizacja jointów
    posOfJoints = getPosOfJoints(1);
    
    % Inicjalizacja czujników ratowników - temperatura cia³a, ciœnienie, puls
    temp = ones(1,numberOfBills)*36.6;
    sys_press = ones(1,numberOfBills)*120;
    dias_press = ones(1,numberOfBills)*80;
    puls = ones(1,numberOfBills)*75;
    prev_sys_press = ones(6000,numberOfBills)*120;
    prev_dias_press = ones(6000,numberOfBills)*80;
    prev_puls = ones(6000,numberOfBills)*75;
    
    % Inicjalizacja zmiennych niezbêdnych do realizacji ruchu ratowników
    k=0;
    dx= zeros(1,numberOfBills);
    dy= zeros(1,numberOfBills);
    avoid = zeros(1,numberOfBills);
    beforAvoid = zeros(numberOfBills,2); %old dx old dy
    avoidStart = zeros(numberOfBills,2);
    
    % Inicjalizacja sygna³ów - wiadomoœci
    signalSpeed = 0.1;
    signalRange = 50;
    r = zeros(1,numberOfBills);
    
    % Wymiary mapy w œrodowisku V-rep
    xMax = 62.5;
    xMin = -17;
    yMax = 38.5;
    yMin = - 40;
    
    % Prêdkoœæ i ustalenie pocz¹tkowych kierunków
    v = (0.009-0.006)*rand(numberOfBills,1)+0.006;
    for i=1:numberOfBills
        if wayOfWalking == 1
            [dx(i),dy(i)] = setPositionFromOrientation(0.1 - 0.2*i,sqrt((v(i)^2)*2));
        else
            dx(i) = v(i);
            dy(i) = -v(i);
        end
    end
    
    % Inicjalizacja wektorów i zmiennych pomocniczych
    injured = zeros(1,numberOfBills);
    stay = zeros(1,numberOfBills);
    inneed = 0;
    flag = 1;
    time = 0;
    saviors = zeros(1,numberOfBills);
    bad = zeros(1,numberOfBills);
    sentCounter = zeros(1,numberOfBills);
    receivedCounter = zeros(1,numberOfBills);
    awaitingMessage = zeros(1,numberOfBills);
    messageForLeader = zeros(1,numberOfBills);
    sentMessages = cell(10,8,numberOfBills);
    receivedMessages = cell(20,6,numberOfBills);
    helpStatus = 0;
    awaitingPartial = zeros(numberOfBills);
    inRange = zeros(numberOfBills);
    routingTable = zeros(numberOfBills);
    whoSent = 0;
    
    fprintf('\n');
    disp('<strong>Symulacja rozpoczêta.</strong>');
    if wayOfWalking==1
        disp('Sposób chodzenia: ratownicy rozchodz¹ siê promieniœcie.');
    else
        disp('Sposób chodzenia: ratownicy rozchodz¹ siê równolegle.');
    end
    if fullNet==1
        disp('Sposób komunikacji: pe³na siatka.');
    else
        disp('Sposób komunikacji: niepe³na siatka (uwzglêdnienie zasiêgu).');
    end
    fprintf('\n');
    %% Pêtla while - pêtla w³aœciwa symulacji
    while flag
        tic
        time = time +1;
        if (mod(time,5) == 0)
            if k ~= 21
                k = k+1;
            else
                k = 1;
            end
            posOfJoints = getPosOfJoints(k);
        end
        
        % W tej pêtli for iterujemy po wszystkich ratownikach
        for i=1:numberOfBills
            % Tutaj sprawdzamy odczyty i okreœlamy rodzaj SOS: 0-brak, 1-SOS, 2-SOS krytyczne
            bad(i) = sosSignalGenerator(temp(i),puls(i),sys_press(i),dias_press(:,i),prev_puls(:,i),prev_sys_press(:,i),prev_dias_press(:,i));
            if( bad(i)>0 && inneed==0 && sum(injured)<1)
                injured(i) = 1;
                inneed = i;
                if i==leader
                    hierarchy = hierarchy-1;
                    leader = find(hierarchy==1);
                end
                if bad(i)==2
                    info = 'SOS_kryt';
                else
                    info = 'SOS';
                end
                if fullNet
                    flyTime = sqrt((positions(leader,1) - positions(i,1))^2 + (positions(leader,2) - positions(i,2))^2);
                    fillMessage(i,sentCounter(i),leader,info,positions(i,1:2),flyTime);
                    sentMessages(sentCounter(i)+1,:,i) = fillMessage(i,sentCounter(i),leader,info,positions(i,1:2),flyTime);
                    sentCounter(i) = sentCounter(i)+1;
                else %czêœciowa siatka
                    for j=1:numberOfBills
                        for l=1:numberOfBills
                            d = sqrt((positions(l,1) - positions(j,1))^2 + (positions(l,2) - positions(j,2))^2);
                            if (d<signalRange && l~=j)
                                inRange(j,l) = d;
                            else
                                inRange(j,l) = 0;
                            end
                        end
                    end
                    %                     d = sqrt((positions(leader,1) - positions(i,1))^2 + (positions(leader,2) - positions(i,2))^2);
                    if inRange(i,leader)
                        flyTime = inRange(i,leader);
                        sentMessages(sentCounter(i)+1,:,i) = fillMessage(i,sentCounter(i),leader,info,positions(i,1:2),flyTime);
                        sentCounter(i) = sentCounter(i)+1;
                        awaitingPartial(leader,i) = sentCounter(i);
                    else
                        
%                         for j=1:numberOFBills
%                             if (j~=i && inRange(i,j))
%                                 flyTime = inRange(i,j);
%                                 sentMessages(sentCounter(i)+1,:,i) = fillPartialMessage(i,sentCounter(i),leader,info,positions(i,1:2),find(inRage(i,:)~=0),flyTime);
%                                 sentCounter(i) = sentCounter(i)+1;
%                                 awaitingPartial(j,i) = sentCounter(i);
%                             end
%                         end
                        
                    end
                end
                fprintf('\n');
                disp('<strong>Uwaga! Wys³ano sygna³ SOS.</strong>');
                name = ['-> Ratownik nr ',num2str(i),' wys³a³ sygna³ SOS.'];
                disp(name);
                vrep.simxSetObjectOrientation(clientID,paramedic(i),-1,[0 -pi/2 0],vrep.simx_opmode_oneshot);
                vrep.simxSetObjectPosition(clientID,paramedic(i),-1,[positions(i,1),positions(i,2),0.1],vrep.simx_opmode_oneshot);
            elseif( bad(i)>0 && injured(i)==0 && sum(injured)>=1)
                temp(i) = 36.6;
                sys_press(i) = 120;
                dias_press(i) = 80;
                puls(i) = 75;
            end
            
            % Jeœli jakiœ ratownik wys³a³ sygna³ SOS - procedura reagowania na sygna³ SOS
            if(inneed)
                if  fullNet
                    fullNetCommunication
                else
                    partialNetCommunication
                end
            end
            
            if(stay(i)==1 || injured(i)==1)
                continue;
            else
                for j=1:4 % Zmiana pozycji nóg
                    vrep.simxSetJointPosition(clientID,joints(i,j),posOfJoints(j),vrep.simx_opmode_streaming);
                end
                
                % Sprawdzanie czy poszkodowany albo inny element znajduje siê w polu widzenia, manewr omijania przeszkody
                [~,detectionState,detectionPoint,detectedObject,~]= vrep.simxReadProximitySensor(clientID,bills_sensor(i),vrep.simx_opmode_streaming);
                if(detectionState)
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
                        fprintf('\n\n');
                        disp('<strong>Sukces!</strong>');
                        name = ['Poszkodowany wykryty przez ratownika nr ',num2str(i),'!'];
                        disp(name);
                        name = ['Ratownik znajduje siê w odleg³oœci ',num2str(norm(detectionPoint)),' od poszkodowanego.'];
                        disp(name)
                        disp('Misja zakoñczona.')
                        flag = 0;
                    end
                end
                
                % Chodzenie w kierunku ratownika potrzebuj¹cego pomocy
                if( saviors(i) ~= 0 )
                    if( abs(positions(i,1) - positions(saviors(i),1)) < 2 && abs(positions(i,2) - positions(saviors(i),2)) < 2)
                        stay(i) = 1;
                        for j=1:4
                            vrep.simxSetJointPosition(clientID,joints(i,j),0,vrep.simx_opmode_oneshot);
                        end
                        disp(['<strong>Ratownik nr ',num2str(i),' dotar³ do poszkodowanego kolegi.</strong>']);
                    end
                    orient2 = setOrientationFromPosition(positions(saviors(i),1) - positions(i,1), positions(saviors(i),2)- positions(i,2));
                    [dx(i),dy(i)] = setPositionFromOrientation(orient2,sqrt((v(i)^2)*2));
                end
                
                % Predykcja nastêpnego po³o¿enia w normalnym przypadku
                if avoid(i) == 0
                    if(saviors(i) == 0 )
                        if positions(i,1) > xMax - abs(dx(i)) && positions(i,1) < xMax + abs(dx(i)) && dx(i)>0
                            dx(i) = -v(i);
                        elseif(positions(i,1) > xMin - abs(dx(i)) && positions(i,1) < xMin + abs(dx(i)) && dx(i) < 0 )
                            dx(i) = v(i);
                        end
                        
                        if positions(i,2) > yMax - abs(dy(i)) && positions(i,2) < yMax + abs(dy(i)) && dy(i)>0
                            dy(i) = -v(i);
                        elseif positions(i,2) > yMin - abs(dy(i)) && positions(i,2) < yMin + abs(dy(i)) && dy(i)<0
                            dy(i) = v(i);
                        end
                    end
                else
                    [dx(i),dy(i),avoid(i)] = avoiding(beforAvoid(i,1),beforAvoid(i,2),avoidStart(i),time,avoid(i));
                end
                
                % Ruch ratowników
                if mod(time,550)==0
                    if rand(1)>0.5
                        positions(i,1)=positions(i,1)+dx(i)*((1.1-0.9)*rand(1)+0.9);
                    else
                        positions(i,2)=positions(i,2)+dy(i)*((1.1-0.9)*rand(1)+0.9);
                    end
                else
                    positions(i,1)=positions(i,1)+dx(i);
                    positions(i,2)=positions(i,2)+dy(i);
                end
                vrep.simxSetObjectPosition(clientID,paramedic(i),-1,[positions(i,1),positions(i,2),positions(i,3)],vrep.simx_opmode_oneshot);
                orient = setOrientationFromPosition(dx(i),dy(i));
                vrep.simxSetObjectOrientation(clientID,paramedic(i),-1,[0 0 orient],vrep.simx_opmode_oneshot);
            end
        end
        
        % Raz na 1000 iteracji jeden losowy ratownik cudownie zdrowieje
        if(mod(time,1000)==0 )
            lucky_guy = randi(numberOfBills,1);
            if(~injured(lucky_guy))
                temp(lucky_guy) = 36.6;
                sys_press(lucky_guy) = 120;
                dias_press(lucky_guy) = 80;
                puls(lucky_guy) = 75;
            end
        end
        
        % 'Losowe' zmienianie wartoœci odczytów z czujników ratowników
        if (mod(time,30) == 0)
            temp = temp + rand(1,numberOfBills)/4 - 0.125;
            sys_press = sys_press + rand(1,numberOfBills)*2 - 1;
            prev_sys_press = [sys_press;prev_sys_press(1:(end-1),:)];
            dias_press = dias_press + rand(1,numberOfBills)*1.3 - 0.6;
            prev_dias_press = [dias_press;prev_dias_press(1:(end-1),:)];
            puls = puls + rand(1,numberOfBills) - 0.5;
            prev_puls = [puls;prev_puls(1:(end-1),:)];
        end
        iterationTime = toc;
        if toc<0.01
            pause(0.01-toc);
        end
    end
    % Zakoñczenie symulacji - ustawienie ratowników w pozycjach stoj¹cych
    for i=1:size(paramedic,2)
        for j=1:4
            vrep.simxSetJointPosition(clientID,joints(i,j),0,vrep.simx_opmode_oneshot);
        end
    end
    pause(2);
    vrep.simxFinish(-1);
else
    disp('Próba po³¹czenia ze œrodowiskiem V-rep nie powiod³a siê.');
end
vrep.delete();