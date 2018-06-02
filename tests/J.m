function [outcome] = J(savior1,savior2,positions,inneed,dead)

alfa1 = 0.5;
active = find(dead==0);

% maxCover = 0;
% for i=1:length(active)
%     maxCover = maxCover + 
% end
% 
% actualCover = 0;
% for i=1:length(active)
%     if i~=savior1 && i~=savior2
%         
%     end
% end

d1 = sqrt((positions(inneed,1) - positions(savior1,1))^2 + (positions(inneed,2) - positions(savior1,2))^2);
d2 = sqrt((positions(inneed,1) - positions(savior1,1))^2 + (positions(inneed,2) - positions(savior1,2))^2);

outcome = alfa1*log((d1+d2)/2);%+alfa2*log(actualCover/maxCover);

end