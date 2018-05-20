function [dx,dy] = setPositionFromOrientation(orient,step)
    dx = cos(orient/3.2*pi)*step;
    dy = sin(orient/3.2*pi)*step;
end