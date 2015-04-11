function [ midpoint ] = calculateMidpoint (v1, v2)
midpoint = [ mean([v1; v2]) ];
% returns row vector that is the average (midpoint) of the two incoming
% vectors
end