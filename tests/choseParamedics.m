function [saviors] = choseParamedics(positions,inneed,injured)

numOfParamedics = length(positions);
optionsMatrix = ones(numOfParamedics);
optionsMatrix = triu(optionsMatrix,1);
optionsMatrix = optionsMatrix - diag(diag(optionsMatrix));

notActive = find(injured==1);
for i=1:length(notActive)
    optionsMatrix(:,notActive(i)) = 0;
    optionsMatrix(notActive(i),:) = 0;
end

[row,col] = find(optionsMatrix==1);
outcomesMatrix = zeros(numOfParamedics);

for  i=1:length(row)
    savior1 = row(i);
    savior2 = col(i);

    outcomesMatrix(savior1,savior2)= J(savior1,savior2,positions,inneed,injured);
end

[bestRow,bestCol] = find(outcomesMatrix==min(outcomesMatrix(outcomesMatrix>0)));
saviors = zeros(numOfParamedics,1);
saviors(bestRow(1))=1;
saviors(bestCol(1))=1;
end