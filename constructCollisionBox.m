function [orientedBox] = constructCollisionBox(currentState, th, corner)
% currentState -> [ x y ] -> x, y for position
% th for heading (th in radians)
% corner -> [ x1 y1; x2 y2; ... ] -> x, y for position

orientedBox = [];

for i=1:length(corner)

    orientedBox = [ orientedBox; currentState + rotateVector(corner(i,:), th) ]; % rotate corner, rotate axle-to-centre offset, add the two, then add to current state (which is on path)
end

end
