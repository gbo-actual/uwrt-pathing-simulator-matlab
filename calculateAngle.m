function [ angle ] = calculateAngle(v) % in degrees
angle = atand(v(1,2)/v(1,1));
end