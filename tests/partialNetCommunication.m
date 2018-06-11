%% Komunikacja w niepe³nej siatce - uwzglêdnienie zasiêgu


if (helpStatus==0 && (i==inneed||i==leader))
    if (sentMessages{sentCounter(inneed),7,inneed}>=signalSpeed && i==inneed)
        sentMessages{sentCounter(inneed),7,inneed} = sentMessages{sentCounter(inneed),7,inneed}-signalSpeed;
    elseif (sentMessages{sentCounter(inneed),7,inneed}<signalSpeed && i==leader)
        helpStatus=1;
        sentMessages{sentCounter(inneed),8,inneed} = 1;
        receivedMessages(receivedCounter(leader)+1,:,leader) = sentMessages(sentCounter(inneed),1:6,i);
        receivedCounter(leader) = receivedCounter(leader)+1;
        disp('-> Lider odebra³ sygna³ SOS.');
        if strcmp('SOS_kryt',signalSOS)
            disp('-> Lider u¿y³ telefonu satelitarnego, aby dodatkowo wezwaæ pomoc z zewn¹trz.');
        end
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
        tempTable = sparse(inRange);
    end
end

if (helpStatus==1 && i==leader)
    stay(:) = 1;
    for l=1:numberOfBills
        for j=1:4
            vrep.simxSetJointPosition(clientID,joints(l,j),0,vrep.simx_opmode_oneshot);
        end
    end
    activeParamedics = ones(1,numberOfBills)-injured;
    for j=1:length(activeParamedics)
        if (j~=i && activeParamedics(j))
            [flyTime,signalRoute,~] = graphshortestpath(tempTable,leader,j);                        
            sentMessages(sentCounter(i)+1,:,i) = fillPartialMessage(i,sentCounter(i),j,'podaj_pozycje',[],j,flyTime);
            awaitingMessage(j) = sentCounter(i)+1;
            sentCounter(i) = sentCounter(i)+1;
        end
    end
    disp('-> Lider wys³a³ komunikaty z proœb¹ o podanie pozycji.');
    helpStatus=2;
end
if (helpStatus==2 && awaitingMessage(i))
    if (sentMessages{awaitingMessage(i),7,leader}>=signalSpeed)
        sentMessages{awaitingMessage(i),7,leader} = sentMessages{awaitingMessage(i),7,leader}-signalSpeed;
    elseif (sentMessages{awaitingMessage(i),7,leader}<signalSpeed)
        sentMessages{awaitingMessage(i),8,leader} = 1;
        receivedMessages(receivedCounter(i)+1,:,i) = sentMessages(awaitingMessage(i),1:6,leader);
        receivedCounter(i) = receivedCounter(i)+1;
        awaitingMessage(i) = 0;
        disp(['-> Ratownik nr ',num2str(i),' odebra³ wiadomoœæ od lidera.']);
        [flyTime,~,~] = graphshortestpath(tempTable,leader,i);
        sentMessages(sentCounter(i)+1,:,i) = fillMessage(i,sentCounter(i),j,'pozycja',positions(i,1:2),flyTime);
        sentCounter(i) = sentCounter(i)+1;
        messageForLeader(i) = sentCounter(i);
        disp(['-> Ratownik nr ',num2str(i),' wys³a³ swoj¹ pozycjê od lidera.']);
    end
elseif (helpStatus==2 && i==leader)
    for j=1:length(messageForLeader)
        if (messageForLeader(j))
            if (sentMessages{messageForLeader(j),7,j}>=signalSpeed)
                sentMessages{messageForLeader(j),7,j} = sentMessages{messageForLeader(j),7,j}-signalSpeed;
            elseif (sentMessages{messageForLeader(j),7,j}<signalSpeed)
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
    saviors = choseParamedics(positions,inneed,injured,dx,dy);
    saviors = saviors*inneed;
    for j=1:length(activeParamedics)
        if (j~=i && activeParamedics(j))
            [flyTime,~,~] = graphshortestpath(tempTable,leader,j);
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
    if (sentMessages{awaitingMessage(i),7,leader}>=signalSpeed)
        sentMessages{awaitingMessage(i),7,leader} = sentMessages{awaitingMessage(i),7,leader}-signalSpeed;
    elseif (sentMessages{awaitingMessage(i),7,leader}<signalSpeed)
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
        disp('<strong>Ratownicy wracaj¹ do swoich zadañ</strong>');
        fprintf('\n');
    end
end