function [map] = readMap(mapPathString)
map = imread(mapPathString);
map = ~im2bw(map);
end