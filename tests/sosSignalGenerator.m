function sossignal = sosSignalGenerator(T,puls,bp,prev_puls,prev_bp)
%SOSSIGNALGENERATOR Summary of this function goes here
%   Detailed explanation goes here
    delta_t = abs(puls - prev_puls(1));
    delta_bp = abs(bp - prev_bp(1,:));
    counter = 0;
    
    if (T >= 34.5 && T < 35.5) || (T > 37.5 && T <= 38.5)
        counter = counter+1;
    elseif T < 34.5 || T > 38.5
        counter = counter+2;
    end
    
    if (t >= 40 && t < 50) || (t > 90 && t <= 120) || (delta_t > 20 && delta_t < 40)
        counter = counter+1;
    elseif t < 40 || t > 120 || delta_t >= 40
        counter = counter+2;
    end
    
    if (t >= 40 && t < 50) || (t > 90 && t <= 120) || (delta_t > 20 && delta_t < 40)
        counter = counter+1;
    elseif t < 40 || t > 120 || delta_t >= 40
        counter = counter+2;
    end
    
    if (bp(1) >= 90 && bp(1) < 100) || (bp(1) > 130 && bp(1) <= 160) || (delta_bp(1) > 20 && delta_bp(1) < 30)
        counter = counter+1;
    elseif bp(1) < 90 || bp(1) > 160 || delta_bp(1) >= 30
        counter = counter+2;
    end
    
    if (bp(2) >= 50 && bp(2) < 70) || (bp(2) > 90 && bp(2) <= 120) || (delta_bp(2) > 10 && delta_bp(2) < 15)
        counter = counter+1;
    elseif bp(2) < 50 || bp(2) > 120 || delta_bp(2) >= 15
        counter = counter+2;
    end

    if counter < 2
        sossignal = 0;
    elseif counter == 2
        sossignal = 1;
    else
        sossignal = 2;
    end
end

