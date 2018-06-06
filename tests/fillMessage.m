function [message] = fillMessage(sender,counter,receiver,info,positions,flyTime)
message = cell(1,8);
message{1,1} = ['R',num2str(sender),'-',num2str(counter+1)];
message{1,2} = info;
message{1,3} = ['R',num2str(sender)];
message{1,4} = ['R',num2str(receiver)];
message{1,5} = positions;
message{1,6} = ['R',num2str(receiver)];
message{1,7} = flyTime;
end