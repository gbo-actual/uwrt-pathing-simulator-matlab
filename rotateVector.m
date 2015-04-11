function [rotatedVector] = rotateVector(v, theta) 
% theta in rad
% v is in ROW form

R = [cos(theta) -sin(theta); sin(theta) cos(theta)];

rotatedVector = v*R;
end