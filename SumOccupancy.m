function [totalOcc] = SumOccupancy(map, startY, startX, sizeY, sizeX)
%CHEECKOCCUPANCY Summary of this function goes here
%   Detailed explanation goes here
% checkOccupancy(map,[xRan yRan]);
tempOccHolder = zeros(sizeY, sizeX);
for i = 1:sizeX
    tempCol = zeros(sizeY, 2);
    for j = 1:sizeY
        tempCol(j,:) = [startY+j-1 startX+i-1];
    end
    tempOccHolder(:,i) = checkOccupancy(map, tempCol, 'grid');      
end
tempOccHolder(tempOccHolder == 1) = 2;
tempOccHolder(tempOccHolder == 0) = 1;

    totalOcc = sum(sum(tempOccHolder));
end
