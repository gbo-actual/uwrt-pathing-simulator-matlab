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
        
        % display the below for debugging loop logic
        % mod(j,numCorner)+1
        % mod(j-1,numCorner)+1
        pointAngle = calculateInternalAngle( ( boxCorner((mod(j,numCorner)+1),:) - obstacle(i,:)), ( boxCorner(mod(j-1,numCorner)+1,:) - obstacle(i,:)) ); % use modulo to loop indices
        internalAngleTotal = internalAngleTotal + pointAngle;
    end
    
    % if angle total is inside of 2pi +/- tol, then the point fails collision check, then set flag to 0 and exit loop
    if ( ( (internalAngleTotal >= (2*pi - pi_tolerance)) && ((internalAngleTotal <= (2*pi + pi_tolerance)) )))
        boxCollisionPass = 0;
    else
        i = i+1; % increment counter manually in lieu of C++ for loop syntax
        boxCollisionPass = 1;
    end
end

% end function
end