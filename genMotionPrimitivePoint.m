function [ outputPoint ] = genMotionPrimitivePoint(currentState,delta,dt,v,L,robotLength)

% currentState is in form [x y th], th measured from y axis
% th IN RADIANS

lastPoint = currentState;

dist_travelled = 0;
outputPoint = [];
stepLength = robotLength/2;
%currentState
%lastPoint
while((dist_travelled <= stepLength) && (length(lastPoint) == 3))
    %th = th + (v/L)*tan(delta)*dt;
    % by adding the formulas except the last positions to primitivePoint in
    % this form, primitivePoint already has the previous values stored.
    % then redefine it as itself being incremented, and that's the final
    % part.
    % it's GENIUS! FUUUUUHAHAHAHAHAHA!
    % a month later I don't get it anymore.
    
    % x and y are swapped below because straight ahead is in positive y
    % direction
    outputPoint = lastPoint + [ -1*v*sin(lastPoint(1,3))*dt, v*cos(lastPoint(1,3))*dt, (v/L)*tan(delta)*dt]; % working in RADIANS right?
    % add incremental distance travelled
    dist_travelled = dist_travelled + calculateDistance(outputPoint(1:2) - lastPoint(1:2));
    lastPoint = outputPoint;
    
end

end