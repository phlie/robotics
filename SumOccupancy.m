function [totalOcc] = SumOccupancy(map, startY, startX, sizeY, sizeX)

% Stores the total sum of the occupancy into an array of the same size
tempOccHolder = zeros(sizeY, sizeX);
for i = 1:sizeX
    tempCol = zeros(sizeY, 2);
    for j = 1:sizeY
        % Stores the Y and X cordinate for the start in an Y x 2 array
        tempCol(j,:) = [startY+j-1 startX+i-1];
    end
    % Uses the cordinate from tempCol to check the occupancy of the whole
    % grid and stores it in the appropriate position
    tempOccHolder(:,i) = checkOccupancy(map, tempCol, 'grid');      
end
% Replace the values of an unknown region with 2, and a free spot as 1
tempOccHolder(tempOccHolder == 1) = 2;
tempOccHolder(tempOccHolder == 0) = 1;

    % Return the total occupancy for the square
    totalOcc = sum(sum(tempOccHolder));
end
