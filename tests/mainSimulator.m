%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
%  Projekt ��czony z MISK i SST                                     %
%  Symulator akcji ratowniczej z wykorzystaniem �rodowiska V-rep    %
%  Jakub Pankiewicz, Konrad Sobolewski, Kinga Staszkiewicz          %
%  Warszawa 2018                                                    %
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

clear all;
vrep=remApi('remoteApi'); % using the prototype file (remoteApiProto.m)
vrep.simxFinish(-1); % just in case, close all opened connections

clientID=vrep.simxStart('127.0.0.1',19999,true,true,5000,5);
wayOfWalking = 1;  % 1-promieni�cie 2-r�wnolegle
numberOfBills = 8; % zmianiam liczbe Bill�w

clc
if (clientID>-1)
    disp('Po��czenie ze �rodowiskiem V-rep udane.');
    paramedic = getRats(numberOfBills,clientID,vrep);   % tworz� 8 ratownik�w
    hierarchy = randperm(length(paramedic));            % hierarchia w�r�d ratownik�w
    leader = find(hierarchy==1);                        % wyb�r lidera
    
    %Poszkodowany - poszukiwany przez ratownik�w
    [~,saveMe]=vrep.simxGetObjectHandle(clientID,'Poszkodowany',vrep.simx_opmode_blocking);
    
    %Jointy ratownik�w - w celu poruszania ko�czynami
    joints = zeros(size(paramedic,2),4); % alokuje macierz na jointy 2x4
    for i=1:size(paramedic,2)
        joints(i,:) = getJoints(i,clientID,vrep); % pobieram jointy ka�dego ratownika
    end
    
    % Inicjalizacja pola widzenia - sensora w V-rep
    bills_sensor = readSensors(clientID,vrep,numberOfBills);
    
    % Inicjalizacja pozycyji ratownik�w
    positions = zeros(size(paramedic,2),3);
    for i=1:size(paramedic,2)
        [~ , position] = vrep.simxGetObjectPosition(clientID,paramedic(i),-1,vrep.simx_opmode_blocking);
        positions(i,:) = position;
    end
    %     positions(5,:)
    
    %Inicjalizacja joint�w
    posOfJoints = getPosOfJoints(1);
    
    %Inicjalizacja czujnik�w ratownik�w - temperatura cia�a, ci�nienie, puls
    temp = ones(1,numberOfBills)*36.6;
    sys_press = ones(1,numberOfBills)*120;
    dias_press = ones(1,numberOfBills)*80;
    puls = ones(1,numberOfBills)*75;
    prev_sys_press = ones(6000,numberOfBills)*120;
    prev_dias_press = ones(6000,numberOfBills)*80;
    prev_puls = ones(6000,numberOfBills)*75;
    
    %Inicjalizacja zmiennych niezb�dnych do realizacji ruchu ratownik�w
    k=0;
    dx= zeros(1,numberOfBills);
    dy= zeros(1,numberOfBills);
    avoid = zeros(1,numberOfBills);
    beforAvoid = zeros(numberOfBills,2); %old dx old dy
    avoidStart = zeros(numberOfBills,2);
    
    %Inicjalizacja sygna��w - wiadomo�ci
    signals = zeros(numberOfBills,numberOfBills); % % od kogo do kogo
    signalSpeed = 0.1;
    r = zeros(1,numberOfBills);
    
    %Wymiary mapy w �rodowisku V-rep
    xMax = 62.5;
    xMin = -17;
    yMax = 38.5;
    yMin = - 40;
    
    %Pr�dko�� i ustalenie pocz�tkowych kierunk�w
    v = 0.008;
    for i=1:numberOfBills
        if wayOfWalking == 1
            [dx(i),dy(i)] = setPositionFromOrientation(0.1 - 0.2*i,sqrt((v^2)*2));
        else
            dx(i) = v;
            dy(i) = -v;
        end
    end
    
    %Inicjalizacja wektor�w i zmiennych pomocniczych
    injured = zeros(1,numberOfBills);
    stay = zeros(1,numberOfBills);
    inneed = 0;
    flag = 1;
    time = 0;
    inRange = zeros(1,numberOfBills);
    saviors = zeros(1,numberOfBills); %kto kogo ratuje
    bad = zeros(1,numberOfBills);
    
    %% P�tla while - p�tla w�a�ciwa symulacji
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
        
        % w tej p�tli for iterujemy po wszystkich ratownikach
        for i=1 : numberOfBills
            
            % tutaj sprawdzamy odczyty i okre�lamy rodzaj SOS: 0-brak, 1-SOS, 2-SOS krytyczne
            bad(i) = sosSignalGenerator(temp(i),puls(i),sys_press(i),dias_press(:,i),prev_puls(:,i),prev_sys_press(:,i),prev_dias_press(:,i));
            
            if( bad(i)>0 && sum(injured)<1)
                injured(i) = 1;
                inneed = i;
                %                r(i) = r(i) + signalSpeed;
                %                d = sqrt((positions(leader,1) - positions(i,1))^2 + (positions(leader,2) - positions(i,2))^2);
                %                if( d < r(i) && inRange(i) == 0 )
                %                    inRange(i) = 1;
                %                    pause(3);
                %                end
                name = ['Ratownik nr ',num2str(i),' wys�a� sygna� SOS.'];
                disp(name);
                if i==leader
                    hierarchy = hierarchy-1;
                    leader = find(hierarchy==1);
                end
            elseif( bad(i)>0 && injured(i)==0 && sum(injured)>=1)
                temp(i) = 36.6;
                sys_press(i) = 120;
                dias_press(i) = 80;
                puls(i) = 75;
            end
            
            % je�li dany ratownik wys�a� sygna� SOS to k�adziemy go i rozpoczynamy procedur� reagowania na sygna� SOS
            if(inneed==i)
                vrep.simxSetObjectOrientation(clientID,paramedic(i),-1,[15 0 0],vrep.simx_opmode_oneshot);
                
                disp('Lider wys�y�a komunikat z pro�b� o podanie swojej pozycji do ratownik�w');
                pause(5);
                disp('Ratownicy odsy�aj� do lidera informacje o swoim po�o�eniu');
                pause(5);
                saviors = choseParamedics(positions,inneed,injured);
                saviors = saviors*inneed
                disp('Lider wysy�a do ratownik�w polecenie: szuka� potrzebuj�cego kolegi albo poszkodowanego');
                pause(30);
                inneed = 0;
                continue;
            end
            
%             if(injured(i)==1)
%                 vrep.simxSetObjectOrientation(clientID,paramedic(i),-1,[0 0 0.15],vrep.simx_opmode_oneshot);
%             end
        
            if(stay(i)==1 || injured(i)==1)
                continue;
            else
                for j=1:4 %zmieniam pozycje n�g
                    vrep.simxSetJointPosition(clientID,joints(i,j),posOfJoints(j),vrep.simx_opmode_streaming);
                end
                        
            %Sprawdzanie czy poszkodowany albo inny element znajduje si� w polu widzenia, manewr omijania przeszkody
            [~,detectionState,detectionPoint,detectedObject,~]= vrep.simxReadProximitySensor(clientID,bills_sensor(i),vrep.simx_opmode_streaming);
            if(detectionState)
                if( detectionPoint(1) < 0.5 && detectionPoint(1) > 0 && detectionPoint(3) < 1.5 && avoid(i)==0) % skr�t w prawo
                    avoid(i) = 1;
                    beforAvoid(i,1) = dx(i);
                    beforAvoid(i,2) = dy(i);
                    avoidStart(i) = time;
                elseif ( detectionPoint(1) > -0.5 && detectionPoint(1) < 0 && detectionPoint(3) < 1.5 && avoid(i)==0) % skr�t w lewo
                    avoid(i) = 2;
                    beforAvoid(i,1) = dx(i);
                    beforAvoid(i,2) = dy(i);
                    avoidStart(i) = time;
                end
                if(saveMe == detectedObject)
                    name = ['Poszkodowany wykryty przez ratownika nr ',num2str(i),'!'];
                    disp(name);
                    name = ['Ratownik znajduje si� w odleg�o�ci ',num2str(norm(detectionPoint)),' od poszkodowanego.'];
                    disp(name)
                    disp('Misja zako�czona sukcesem.')
                    flag = 0;
                end
            end
            
            % chodzenie w kierunku ratownika potrzebuj�cego pomocy
            if( saviors(i) ~= 0 )
                if( abs(positions(i,1) - positions(saviors(i),1)) < 1 && abs(positions(i,2) - positions(saviors(i),2)) < 1)
                    stay(i) = 1;
                end
                orient2 = setOrientationFromPosition(positions(saviors(i),1) - positions(i,1), positions(saviors(i),2)- positions(i,2));
                [dx(i),dy(i)] = setPositionFromOrientation(orient2,sqrt((v^2)*2));
            end
            
            %predykcja nast�pnego po�o�enia w normalnym przypadku
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
            
            % Ruch ratownik�w
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
        
        % 'losowe' zmienianie warto�ci odczyt�w z czujnik�w ratownik�w
        if (mod(time,30) == 0)
            temp = temp + rand(1,numberOfBills)/4 - 0.125;
            sys_press = sys_press + rand(1,numberOfBills)*2 - 1;
            prev_sys_press = [sys_press;prev_sys_press(1:(end-1),:)];
            dias_press = dias_press + rand(1,numberOfBills)*1.3 - 0.6;
            prev_dias_press = [dias_press;prev_dias_press(1:(end-1),:)];
            puls = puls + rand(1,numberOfBills) - 0.5;
            prev_puls = [puls;prev_puls(1:(end-1),:)];
        end
    end
    % Zako�czenie symulacji - ustawienie ratownik�w w pozycjach stoj�cych
    for i=1 : size(paramedic,2)
        for j=1:4
            vrep.simxSetJointPosition(clientID,joints(i,j),0,vrep.simx_opmode_oneshot);
        end
    end
    
    vrep.simxFinish(-1);
else
    disp('Pr�ba po��czenia ze �rodowiskiem V-rep nie powiod�a si�.');
end
vrep.delete();