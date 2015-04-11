%vectors

function [ angle ] = calculateAngle(v) % in degrees
angle = atand(v(1,2)/v(1,1));
end

function [rotatedVector] = rotateVector(v, angle) % angle in degrees
% v is in ROW form
rotatedVector = [cosd(angle) -sind(angle); sind(angle) cosd(angle)] * v';
end

function [boxCollisionPass] = collisionCheck(boxCorner, obstacle)
% individual vector entries are in ROW form, entries are entered in
% separate rows.
% boxCorner and obstacle matrices are n rows by 2 columns
% 4 corners for boxCorner, so numCorner should be 4 anyway
numObst = length(obstacle);
numCorner = length(boxCorner);
pi_tolerance = degtorad(0.1);
i = 1;
boxCollisionPass = 1;
while ((i <= numObst) && (boxCollisionPass == 1))
 % loop for obstacle points    
    internalAngleTotal = 0;
    for j=1:numCorner % loop for box corners
        pointAngle = CalculateAngle(obstacle(i,:) - boxCorner(mod(j,4)+1,:), obstacle(i,:) - boxCorner(mod(j-1,4)+1,:)); % use modulo to loop indices
        internalAngleTotal = internalAngleTotal + pointAngle;
    end
    
    % if angle total is outside of 2pi +/- tol, then the point fails collision check, then set flag to 0 and exit loop
    if ((internalAngleTotal <= (2*pi - pi_tolerance) || (internalAngleTotal >= (2*pi + pi_tolerance))))
        boxCollisionPass = 0;
    end
i = i+1; % increment counter manually in lieu of C++ for loop syntax
end

% end function
end

function [ Distance ] = calculateDistance(v)
Distance = norm(v);

end

function [ midpoint ] = calculateMidpoint (v1, v2)
midpoint = [ mean([v1; v2]) ];
% returns row vector that is the average (midpoint) of the two incoming
% vectors
end

function [ Angle ]= calculateInternalAngle(A,B)

C=dot(A,B);
CosAngle= dot(A,B)/(norm(A)*norm(B));
Angle = acos(CosAngle); % the angle is in radians.

end

function [ Energy ]= calculatePotentialEnergy(CurrentPoint, obstacle)

numObstPoint = length(obstacle);
Energy = 0;

for (i=1:numObstPoint)

    distance = distance + calculateDistance(obstacle(i) - CurrentPoint);
    Energy = Energy + 1/(calculateDistance(obstacle(i) - CurrentPoint))^2;
end

end

function [ MinDistance ]= minimumDistance(obstacle, CurrentPoint)

DistanceArray = []; % allocate array

numObst = length(obstacle);

% continuously add distances from obstacles to point
for i=1:numObst
    DistanceArray = [DistanceArray; calculateDistance(obstacle(i,:)-CurrentPoint)];
end

% take min of array
MinDistance = min(DistanceArray);
end

function [ mindistvar] = calculateMinDistVariance(minDistArray)
mindistvar = 0;
maxMinDist = max(minDistArray);

for i=1:length(minDistArray)
    mindistvar = mindistvar + (maxMinDist - minDistArray(i))^2; % does not take into account total number of points in minDistArray (ie. not per capita)
end


end
