function [ MinDistance ]= minimumDistance(obstacle, CurrentPoint)

DistanceArray = []; % allocate array

numObst = length(obstacle);

% continuously add distances from obstacles to point
for i=1:numObst
    DistanceArray = [DistanceArray; calculateDistance(obstacle(i,:) - CurrentPoint)];
end

% take min of array
MinDistance = min(DistanceArray);
end