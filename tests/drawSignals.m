function [] = drawSignals(positions,signalRoute,signalRange,inneed,leader)

% Wymiary mapy
xMax = 62.5;
xMin = -17;
yMax = 38.5;
yMin = - 40;


figure;
hold on
for i=1:length(positions)
    if i~=inneed && i~=leader
        plot(positions(i,1),positions(i,2),'.r','MarkerSize',10);
        text(positions(i,1)+1,positions(i,2)+1,num2str(i),'Color','k');
    end
    if i==inneed
        plot(positions(inneed,1),positions(inneed,2),'.b','MarkerSize',15);
        text(positions(inneed,1)+1,positions(inneed,2)+1,num2str(inneed),'Color','k');
    end
    if i==leader
        plot(positions(i,1),positions(i,2),'.r','MarkerSize',10);
        text(positions(i,1)+1,positions(i,2)+1,[num2str(i),'(lider)'],'Color','k');
    end
end
grid on;
th = 0:pi/50:2*pi;

for i=1:length(signalRoute)-1
        plot([positions(signalRoute(i),1),positions(signalRoute(i+1),1)],[positions(signalRoute(i),2),positions(signalRoute(i+1),2)],'r');  
        xunit = signalRange * cos(th) + positions(signalRoute(i),1);
        yunit = signalRange * sin(th) + positions(signalRoute(i),2);
        plot(xunit, yunit,'g');
end

xlim([xMin-10 xMax+10]);
ylim([yMin-10 yMax+10]);

title('Trasa syga³u SOS od potrzebuj¹cego do lidera z uwzglêdnieniem zasiêgu');

h = zeros(2, 1);
h(1) = plot(NaN,NaN,'.r','MarkerSize',10, 'visible', 'on');
h(2) = plot(NaN,NaN,'.b','MarkerSize',15, 'visible', 'on');
legend(h,'ratownik','ratownik, który wys³a³ SOS','Location','best');
end