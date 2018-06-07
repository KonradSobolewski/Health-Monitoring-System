function [outcome] = J(savior1,savior2,positions,inneed,injured,dx,dy)

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

d1 = sqrt((positions(inneed,1) - positions(savior1,1))^2 + (positions(inneed,2) - positions(savior1,2))^2);
d2 = sqrt((positions(inneed,1) - positions(savior1,1))^2 + (positions(inneed,2) - positions(savior1,2))^2);

%Wynik
outcome = alfa1*((d1+d2)/2)+alfa2*(1-actualCover/maxCover);

end