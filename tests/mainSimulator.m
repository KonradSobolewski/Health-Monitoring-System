%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Projekt ³¹czony z MISK i SST                                     %
%  Symulator akcji ratowniczej z wykorzystaniem œrodowiska V-rep    %
%  Jakub Pankiewicz, Konrad Sobolewski, Kinga Staszkiewicz          %
%  Warszawa 2018                                                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections

clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
wayOfWalking = 1;  % 1-promieniœcie 2-równolegle
fullNet = 1; % 1-pe³na siatka, 0-niepe³na siatka
numberOfBills = 8; % zmianiam liczbe Billów

clc
if (clientID>-1)
    disp('Po³¹czenie ze œrodowiskiem V-rep udane.');
    disp('Trwa inicjalizacja systemu...');
    paramedic = getRats(numberOfBills,clientID,vrep);   % tworzê 8 ratowników
    hierarchy = randperm(length(paramedic));            % hierarchia wœród ratowników
    leader = find(hierarchy==1);                        % wybór lidera
    
    %Poszkodowany - poszukiwany przez ratowników
    [~,saveMe]=vrep.simxGetObjectHandle(clientID,'Poszkodowany',vrep.simx_opmode_blocking);
    
    %Jointy ratowników - w celu poruszania koñczynami
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
    
    %Inicjalizacja jointów
    posOfJoints = getPosOfJoints(1);
    
    %Inicjalizacja czujników ratowników - temperatura cia³a, ciœnienie, puls
    temp = ones(1,numberOfBills)*36.6;
    sys_press = ones(1,numberOfBills)*120;
    dias_press = ones(1,numberOfBills)*80;
    puls = ones(1,numberOfBills)*75;
    prev_sys_press = ones(6000,numberOfBills)*120;
    prev_dias_press = ones(6000,numberOfBills)*80;
    prev_puls = ones(6000,numberOfBills)*75;
    
    %Inicjalizacja zmiennych niezbêdnych do realizacji ruchu ratowników
    k=0;
    dx= zeros(1,numberOfBills);
    dy= zeros(1,numberOfBills);
    avoid = zeros(1,numberOfBills);
    beforAvoid = zeros(numberOfBills,2); %old dx old dy
    avoidStart = zeros(numberOfBills,2);
    
    %Inicjalizacja sygna³ów - wiadomoœci
    signals = zeros(numberOfBills,numberOfBills); % % od kogo do kogo
    signalSpeed = 0.1;
    r = zeros(1,numberOfBills);
    
    %Wymiary mapy w œrodowisku V-rep
    xMax = 62.5;
    xMin = -17;
    yMax = 38.5;
    yMin = - 40;
    
    %Prêdkoœæ i ustalenie pocz¹tkowych kierunków
    v = 0.008;
    for i=1:numberOfBills
        if wayOfWalking == 1
            [dx(i),dy(i)] = setPositionFromOrientation(0.1 - 0.2*i,sqrt((v^2)*2));
        else
            dx(i) = v;
            dy(i) = -v;
        end
    end
    
    %Inicjalizacja wektorów i zmiennych pomocniczych
    injured = zeros(1,numberOfBills);
    stay = zeros(1,numberOfBills);
    inneed = 0;
    flag = 1;
    time = 0;
    inRange = zeros(1,numberOfBills);
    saviors = zeros(1,numberOfBills); %kto kogo ratuje
    bad = zeros(1,numberOfBills);
    sentCounter = zeros(1,numberOfBills);
    receivedCounter = zeros(1,numberOfBills);
    awaitingMessage = zeros(1,numberOfBills);
    messageForLeader = zeros(1,numberOfBills);
    sentMessages = cell(10,8,numberOfBills);
    receivedMessages = cell(20,6,numberOfBills);
    helpStatus = 0;
    
    disp('Symulacja rozpoczêta.');
    fprintf('\n');
    %% Pêtla while - pêtla w³aœciwa symulacji
    while flag
        tic
        time = time +1;
        if (mod(time,5) == 0)
            if k ~= 21 % size of position arrays
                k = k+1;
            else
                k = 1;
            end
            posOfJoints = getPosOfJoints(k);
        end
        
        % w tej pêtli for iterujemy po wszystkich ratownikach
        for i=1:numberOfBills
            % tutaj sprawdzamy odczyty i okreœlamy rodzaj SOS: 0-brak, 1-SOS, 2-SOS krytyczne
            bad(i) = sosSignalGenerator(temp(i),puls(i),sys_press(i),dias_press(:,i),prev_puls(:,i),prev_sys_press(:,i),prev_dias_press(:,i));
            if( bad(i)>0 && inneed==0 && sum(injured)<1)
                injured(i) = 1;
                inneed = i;
                if i==leader
                    hierarchy = hierarchy-1;
                    leader = find(hierarchy==1);
                end
                if fullNet
                    if bad(i)==2
                        info = 'SOS_kryt';
                    else
                        info = 'SOS';
                    end
                    flyTime = sqrt((positions(leader,1) - positions(i,1))^2 + (positions(leader,2) - positions(i,2))^2);
                    fillMessage(i,sentCounter(i),leader,info,positions(i,1:2),flyTime)
                    sentMessages(sentCounter(i)+1,:,i) = fillMessage(i,sentCounter(i),leader,info,positions(i,1:2),flyTime);
                    sentCounter(i) = sentCounter(i)+1;
                else
                    %                r(i) = r(i) + signalSpeed;
                    %                d = sqrt((positions(leader,1) - positions(i,1))^2 + (positions(leader,2) - positions(i,2))^2);
                    %                if( d < r(i) && inRange(i) == 0 )
                    %                    inRange(i) = 1;
                    %                    pause(3);
                    %                end
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
            
            % jeœli jakiœ ratownik wys³a³ sygna³ SOS - rozpoczynamy procedurê reagowania na sygna³ SOS
            if(inneed)
                if (helpStatus==0 && (i==inneed||i==leader))
                    if (sentMessages{sentCounter(inneed),7,inneed}>=0.1 && i==inneed)
                        sentMessages{sentCounter(inneed),7,inneed} = sentMessages{sentCounter(inneed),7,inneed}-0.1;
                    elseif (sentMessages{sentCounter(inneed),7,inneed}<0.1 && i==leader)
                        helpStatus=1;
                        sentMessages{sentCounter(inneed),8,inneed} = 1;
                        receivedMessages(receivedCounter(leader)+1,:,leader) = sentMessages(sentCounter(inneed),1:6,i);
                        receivedCounter(leader) = receivedCounter(leader)+1;
                        disp('-> Lider odebra³ sygna³ SOS.');
                    end
                end
                if (helpStatus==1 && i==leader)
                    stay(:) = 1;
                    activeParamedics = ones(1,numberOfBills)-injured;
                    for j=1:length(activeParamedics)
                        if (j~=i && activeParamedics(j))
                            flyTime = sqrt((positions(leader,1) - positions(j,1))^2 + (positions(leader,2) - positions(j,2))^2);
                            sentMessages(sentCounter(i)+1,:,i) = fillMessage(i,sentCounter(i),j,'podaj_pozycje',[],flyTime);
                            awaitingMessage(j) = sentCounter(i)+1;
                            sentCounter(i) = sentCounter(i)+1;
                        end
                    end
                    disp('-> Lider wys³a³ komunikaty z proœb¹ o podanie pozycji.');
                    helpStatus=2;
                end
                if (helpStatus==2 && awaitingMessage(i))
                    if (sentMessages{awaitingMessage(i),7,leader}>=0.1)
                        sentMessages{awaitingMessage(i),7,leader} = sentMessages{awaitingMessage(i),7,leader}-0.1;
                    elseif (sentMessages{awaitingMessage(i),7,leader}<0.1)
                        sentMessages{awaitingMessage(i),8,leader} = 1;
                        receivedMessages(receivedCounter(i)+1,:,i) = sentMessages(awaitingMessage(i),1:6,leader);
                        receivedCounter(i) = receivedCounter(i)+1;
                        awaitingMessage(i) = 0;
                        disp(['-> Ratownik nr ',num2str(i),' odebra³ wiadomoœæ od lidera.']);
                        flyTime = sqrt((positions(leader,1) - positions(i,1))^2 + (positions(leader,2) - positions(i,2))^2);
                        sentMessages(sentCounter(i)+1,:,i) = fillMessage(i,sentCounter(i),j,'pozycja',positions(i,1:2),flyTime);
                        sentCounter(i) = sentCounter(i)+1;
                        messageForLeader(i) = sentCounter(i);
                        disp(['-> Ratownik nr ',num2str(i),' wys³a³ swoj¹ pozycjê od lidera.']);
                    end
                elseif (helpStatus==2 && i==leader)
                    for j=1:length(messageForLeader)
                        if (messageForLeader(j))
                            if (sentMessages{messageForLeader(j),7,j}>=0.1)
                                sentMessages{messageForLeader(j),7,j} = sentMessages{messageForLeader(j),7,j}-0.1;
                            elseif (sentMessages{messageForLeader(j),7,j}<0.1)
                                sentMessages{messageForLeader(j),8,j} = 1;
                                receivedMessages(receivedCounter(leader)+1,:,i) = sentMessages(messageForLeader(j),1:6,j);
                                receivedCounter(leader) = receivedCounter(leader)+1;
                                messageForLeader(j) = 0;
                                disp(['-> Lider odebra³ pozycjê od ratownika nr ',num2str(j),'.']);
                            end
                        end
                    end
                    if sum(messageForLeader+awaitingMessage)==0
                        helpStatus = 3;
                    end
                end
                if (helpStatus==3 && i==leader)
                    disp('-> Lider odebra³ pozycje od wszystkich ratowników.');
                    saviors = choseParamedics(positions,inneed,injured);
                    saviors = saviors*inneed;
                    for j=1:length(activeParamedics)
                        if (j~=i && activeParamedics(j))
                            flyTime = sqrt((positions(leader,1) - positions(j,1))^2 + (positions(leader,2) - positions(j,2))^2);
                            if saviors(j)
                                sentMessages(sentCounter(i)+1,:,i) = fillMessage(i,sentCounter(i),j,'idz',positions(inneed,1:2),flyTime);
                            else
                                sentMessages(sentCounter(i)+1,:,i) = fillMessage(i,sentCounter(i),j,'szukaj',[],flyTime);
                            end
                            sentCounter(i) = sentCounter(i)+1;
                            awaitingMessage(j) = sentCounter(i);
                        end
                    end
                    tempSav = find(saviors>0);
                    disp('-> Lider wysy³a do ratowników swoj¹ decyzjê.');
                    disp(['-> Pomóc koledze id¹ ratownicy nr ',num2str(tempSav(1)),' oraz ',num2str(tempSav(2)),'.']);
                    stay(leader) = 0;
                    helpStatus = 4;
                end
                if (helpStatus==4 && awaitingMessage(i))
                    if (sentMessages{awaitingMessage(i),7,leader}>=0.1)
                        sentMessages{awaitingMessage(i),7,leader} = sentMessages{awaitingMessage(i),7,leader}-0.1;
                    elseif (sentMessages{awaitingMessage(i),7,leader}<0.1)
                        sentMessages{awaitingMessage(i),8,leader} = 1;
                        receivedMessages(receivedCounter(i)+1,:,i) = sentMessages(awaitingMessage(i),1:6,leader);
                        receivedCounter(i) = receivedCounter(i)+1;
                        awaitingMessage(i) = 0;
                        disp(['-> Ratownik nr ',num2str(i),' odebra³ wiadomoœæ od lidera.']);
                        stay(i) = 0;
                    end
                    if sum(awaitingMessage)==0
                        helpStatus = 0;
                        inneed = 0;
                    end
                end
                %                 pause(5);
                %                 disp('-> Ratownicy odsy³aj¹ do lidera informacje o swoim po³o¿eniu.');
                %                 pause(5);
                %                 saviors = choseParamedics(positions,inneed,injured);
                %                 saviors = saviors*inneed;
                %                 disp('-> Lider wysy³a do ratowników swoj¹ decyzjê.');
                %                 pause(30);
                %                 inneed = 0;
            end
            
            if(stay(i)==1 || injured(i)==1)
                continue;
            else
                for j=1:4 %zmieniam pozycje nóg
                    vrep.simxSetJointPosition(clientID,joints(i,j),posOfJoints(j),vrep.simx_opmode_streaming);
                end
                
                %Sprawdzanie czy poszkodowany albo inny element znajduje siê w polu widzenia, manewr omijania przeszkody
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
                
                % chodzenie w kierunku ratownika potrzebuj¹cego pomocy
                if( saviors(i) ~= 0 )
                    if( abs(positions(i,1) - positions(saviors(i),1)) < 2 && abs(positions(i,2) - positions(saviors(i),2)) < 2)
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
                
                % Ruch ratowników
                positions(i,1)=positions(i,1)+dx(i);
                positions(i,2)=positions(i,2)+dy(i);
                vrep.simxSetObjectPosition(clientID,paramedic(i),-1,[positions(i,1),positions(i,2),positions(i,3)],vrep.simx_opmode_oneshot);
                orient = setOrientationFromPosition(dx(i),dy(i));
                vrep.simxSetObjectOrientation(clientID,paramedic(i),-1,[0 0 orient],vrep.simx_opmode_oneshot);
            end
        end
        
        % raz na 1000 iteracji jeden losowy ratownik cudownie zdrowieje
        if(mod(time,1000)==0 )
            lucky_guy = randi(numberOfBills,1);
            if(~injured(lucky_guy))
                temp(lucky_guy) = 36.6;
                sys_press(lucky_guy) = 120;
                dias_press(lucky_guy) = 80;
                puls(lucky_guy) = 75;
            end
        end
        
        % 'losowe' zmienianie wartoœci odczytów z czujników ratowników
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
    
    vrep.simxFinish(-1);
else
    disp('Próba po³¹czenia ze œrodowiskiem V-rep nie powiod³a siê.');
end
vrep.delete();