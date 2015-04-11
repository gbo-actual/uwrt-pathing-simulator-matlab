function [collisionDetected] = circularCollisionDetection(currentPosition, obstacle)
% currentPosition -> [x y]
% obstacle -> [x1 y1; x2 y2; ... ]
collisionDetected = false;
i = 1;
radius = 50; % cm; definitely not properly estimating pylon spacing
while ((i <= length(obstacle)) && (collisionDetected == 0))
    if ((calculateDistance(obstacle(i,:) - currentPosition)) <= radius)
        collisionDetected = true;
    else
        i = i + 1;
    end
    
end

end