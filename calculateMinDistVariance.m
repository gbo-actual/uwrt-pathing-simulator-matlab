function [ mindistvar] = calculateMinDistVariance(minDistArray)
mindistvar = 0;
maxMinDist = max(minDistArray);

for i=1:length(minDistArray)
        mindistvar = mindistvar + (maxMinDist - minDistArray(i))^2; % does not take into account total number of points in minDistArray (ie. not per capita)
end

end