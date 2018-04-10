function pos = getPosOfJoints(k)
    leftLegWaypoints=[0.237,0.228,0.175,-0.014,-0.133,-0.248,-0.323,-0.450,-0.450,...
         -0.442,-0.407,-0.410,-0.377,-0.303,-0.178,-0.111,-0.010,0.046,0.104,0.145,0.188];
    rightLegWaypoints=[-0.442,-0.407,-0.410,-0.377,-0.303,-0.178,-0.111,-0.010,0.046,0.104,...
         0.145,0.188,0.237,0.228,0.175,-0.014,-0.133,-0.248,-0.323,-0.450,-0.450];

    leftKneeWaypoints=[0.282,0.403,0.577,0.929,1.026,1.047,0.939,0.664,0.440,0.243,...
         0.230,0.320,0.366,0.332,0.269,0.222,0.133,0.089,0.065,0.073,0.092];
    rightKneeWaypoints=[0.230,0.320,0.366,0.332,0.269,0.222,0.133,0.089,0.065,0.073,0.092,...
         0.282,0.403,0.577,0.929,1.026,1.047,0.939,0.664,0.440,0.243];
    
    pos = [leftLegWaypoints(k), rightLegWaypoints(k), leftKneeWaypoints(k),rightKneeWaypoints(k)];
end