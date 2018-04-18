function orient = setOrientationFromPosition(dx,dy)
    if dx == 0
        if dy > 0
            orient = 1.6;
        else
            orient = -1.6;
        end
    elseif dy == 0
        if dx > 0
            orient = 0;
        else
            orient = 3.2;
        end
    else
        if dx > 0
            orient = acot(dy/dx)*3.2/pi;
        else
            orient = sign(dy)*(3.2-abs(acot(dy/dx)*3.2/pi));
        end
    end        
end

