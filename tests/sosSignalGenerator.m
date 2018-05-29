function sossignal = sosSignalGenerator(T,puls,sys_bp,dias_bp,prev_puls,prev_sys_bp,prev_dias_bp)
%SOSSIGNALGENERATOR Summary of this function goes here
%   Detailed explanation goes here
    delta_t = abs(puls - prev_puls(1));
    delta_sys_bp = abs(sys_bp - prev_sys_bp(1,:));
    delta_dias_bp = abs(dias_bp - prev_dias_bp(1,:));
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
    
    if (sys_bp >= 90 && sys_bp < 100) || (sys_bp > 130 && sys_bp <= 160) || (delta_sys_bp > 20 && delta_sys_bp < 30)
        counter = counter+1;
    elseif sys_bp < 90 || sys_bp > 160 || delta_sys_bp >= 30
        counter = counter+2;
    end
    
    if (dias_bp >= 50 && dias_bp < 70) || (dias_bp > 90 && dias_bp <= 120) || (delta_dias_bp > 10 && delta_dias_bp < 15)
        counter = counter+1;
    elseif dias_bp < 50 || dias_bp > 120 || delta_dias_bp >= 15
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

