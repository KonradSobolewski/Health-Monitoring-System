function [] = drawCover(savior1,savior2,positions,inneed,injured,dx,dy)

% Wymiary mapy
xMax = 62.5;
xMin = -17;
yMax = 38.5;
yMin = - 40;

% Wspó³czynniki
alfa1 = 0.5;
alfa2 = 0.5*10;

% Pole pokrycia terenu jednego ratownika
onesWidth = 5;
onesLength = 8;

% Pokrycie terenu
saviors = zeros(1,length(positions));
saviors(savior1) = 1;
saviors(savior2) = 1;

active = find((injured+saviors)==0);

corner = zeros(length(active),4,2);

maxCover = 0;
for i=1:length(active)
    maxCover = maxCover + onesWidth*onesLength;
end

actualCover = 0;
for i=1:length(active)
    orient = setOrientationFromPosition(dx(i),dy(i));
    [corner(i,1,1),corner(i,1,2)] = setPositionFromOrientation(orient+3.2/2,onesWidth/2);
    [corner(i,2,1),corner(i,2,2)] = setPositionFromOrientation(orient-3.2/2,onesWidth/2);
    [x,y] = setPositionFromOrientation(orient,onesLength);
    corner(i,4,1) = corner(i,1,1) + x;
    corner(i,4,2) = corner(i,1,2) + y;
    corner(i,3,1) = corner(i,2,1) + x;
    corner(i,3,2) = corner(i,2,2) + y;
    corner(i,:,1) = corner(i,:,1) + positions(active(i),1);
    corner(i,:,2) = corner(i,:,2) + positions(active(i),2);
    
end

for i=1:length(active)
    if i==1
        [x,y] = poly2cw([corner(i,1,1),corner(i,2,1),corner(i,3,1),corner(i,4,1)],[corner(i,1,2),corner(i,2,2),corner(i,3,2),corner(i,4,2)]);
    else
        [x1,y1] = poly2cw([corner(i,1,1),corner(i,2,1),corner(i,3,1),corner(i,4,1)],[corner(i,1,2),corner(i,2,2),corner(i,3,2),corner(i,4,2)]);
        [x,y] = polybool('union',x,y,x1,y1);
    end
end

counter = length(find(isnan(x)))+1;
breakPoints(1) = 0;
breakPoints(2:counter) = find(isnan(x));
breakPoints(counter+1) = length(x)+1;

for i=1:counter
    actualCover = actualCover + polyarea(x(breakPoints(i)+1:breakPoints(i+1)-1),y(breakPoints(i)+1:breakPoints(i+1)-1));
end

% actualCover
% maxCover

figure;
plot(x,y);
grid on;
xlim([xMin-10 xMax+10]);
ylim([yMin-10 yMax+10]);

[f, v] = poly2fv(x,y);
patch('Faces',f,'Vertices',v,'FaceColor','g','EdgeColor','none');
hold on
for i=1:length(active)
    plot(positions(active(i),1),positions(active(i),2),'.k','MarkerSize',10);
    text(positions(active(i),1)+1,positions(active(i),2),num2str(active(i)),'Color','k');
end

plot([positions(savior1,1),positions(inneed,1)],[positions(savior1,2),positions(inneed,2)],'r');
plot([positions(savior2,1),positions(inneed,1)],[positions(savior2,2),positions(inneed,2)],'r');

plot(positions(inneed,1),positions(inneed,2),'.b','MarkerSize',15);
text(positions(inneed,1)+1,positions(inneed,2)+1,num2str(inneed),'Color','k');

plot(positions(savior1,1),positions(savior1,2),'.r','MarkerSize',10);
text(positions(savior1,1)+1,positions(savior1,2)+1,num2str(savior1),'Color','k');
plot(positions(savior2,1),positions(savior2,2),'.r','MarkerSize',10);
text(positions(savior2,1)+1,positions(savior2,2)+1,num2str(savior2),'Color','k');

title('Pokrycie terenu przez pozosta³ych ratowników');

h = zeros(3, 1);
h(1) = plot(NaN,NaN,'.k','MarkerSize',10, 'visible', 'on');
h(2) = plot(NaN,NaN,'.r','MarkerSize',10, 'visible', 'on');
h(3) = plot(NaN,NaN,'.b','MarkerSize',15, 'visible', 'on');
legend(h, 'szukaj¹cy poszkodowanego','id¹cy do kolegi','potrzebuj¹cy kolega','Location','best');
end