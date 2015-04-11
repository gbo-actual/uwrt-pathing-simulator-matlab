function [ Energy ]= calculatePotentialEnergy(CurrentPoint, obstacle)

numObstPoint = length(obstacle);
Energy = 0;

for (i=1:numObstPoint)

    Energy = Energy + 1/(calculateDistance(obstacle(i) - CurrentPoint))^2;
end

end