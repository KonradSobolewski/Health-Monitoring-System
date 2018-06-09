function [ dx,dy,avoid] = avoiding( dxStart,dyStart,timeStart,time,avoid )
%avoid - czy dalej omijam
%reszta - nowe prêdkoœci
% avoid  - 1 lewo, 2 prawo
    dx = dxStart;
    dy = dyStart;
    if(time -timeStart  >= 0 && time -timeStart  < 150)
        if avoid == 1
            orient = setOrientationFromPosition(dxStart,dyStart);
            [dx,dy] = setPositionFromOrientation(orient-0.8,sqrt(dxStart^2 + dyStart^2));
        else
            orient = setOrientationFromPosition(dxStart,dyStart);
            [dx,dy] = setPositionFromOrientation(orient+0.8,sqrt(dxStart^2 + dyStart^2));
        end
    elseif (time -timeStart  >= 150 && time -timeStart < 270)
        dx = dxStart;
        dy = dyStart;
    elseif (time -timeStart >= 270 && time -timeStart  < 430)
        if avoid == 1
            orient = setOrientationFromPosition(dxStart,dyStart);
            [dx,dy] = setPositionFromOrientation(orient+0.8,sqrt(dxStart^2 + dyStart^2));
        else
            orient = setOrientationFromPosition(dxStart,dyStart);
            [dx,dy] = setPositionFromOrientation(orient-0.8,sqrt(dxStart^2 + dyStart^2));
        end
    elseif (time -timeStart  >= 430 )
        dx = dxStart;
        dy = dyStart;
        avoid = 0;
    end
end

