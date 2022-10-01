function [regionToGo, maxRegion] = DivideMap(map, sizeY, sizeX, resolution, counter)


% Size of the map and initilization
divideIntoX = 26;
divideIntoY = 21;
zones = zeros(divideIntoY*divideIntoX, 2);

% Divides the map into 1m x 1m squares
divisionX = (sizeX * resolution) / divideIntoX;
divisionY = (sizeY * resolution) / divideIntoY;

% For the total amount of squares on the map
for j = 1:divideIntoX
    for i = 1:divideIntoY
         % Get the cordinates for the zones in a Y by 2 array
         zones(i + divideIntoY*(j-1),:) = [1*divisionX*(j-1)+1, 1*divisionY*(i-1)+1];
    end
end

% Calculates the total sum of the occupancy map for each square
temp = zeros(divideIntoY, divideIntoX);
for i = 1:divideIntoY
    for j = 1:divideIntoX
        % SumOccupnacy takes in the map and the current square to sum the
        % occupancy for and returns a sum indicating the known status of
        % that square
        temp(i,j) = SumOccupancy(map, zones(i + divideIntoY*(j-1),2), zones(i + divideIntoY*(j-1),1), divisionY, divisionX);
    end
end

% More initilization
temp
size(temp)
counter
holder = 0;
threshold = -360+25*counter;        % A calculation to determine what the minimium threshold is
posj = 0;
posi = 0;

% Simply loop over the squares and decide which one has the lowest threshold 
for i = 1:divideIntoX
    for j = 1:divideIntoY
        if (holder > temp(j,i) && temp(j,i) > threshold)
            holder = temp(j,i);
            posj = j;
            posi = i;
        end
       
    end
end
holder;
posj;
posi;

% Return the square on the map to explore next
regionToGo = [(divideIntoY-posj) posi-1]
% maxRegion = [(divideInto-posj+1)*divisionX ((posi+1)*divisionY -1)]
end

