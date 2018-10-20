function [regionToGo, maxRegion] = DivideMap(map, sizeY, sizeX, resolution, counter)
%DIVIDEMAP Summary of this function goes here
%   Detailed explanation goes here
divideIntoX = 26;
divideIntoY = 21;
zones = zeros(divideIntoY*divideIntoX, 2);
divisionX = (sizeX * resolution) / divideIntoX;
divisionY = (sizeY * resolution) / divideIntoY;

for j = 1:divideIntoX
    for i = 1:divideIntoY
         zones(i + divideIntoY*(j-1),:) = [1*divisionX*(j-1)+1, 1*divisionY*(i-1)+1];
    end
end
temp = zeros(divideIntoY, divideIntoX);
for i = 1:divideIntoY
    for j = 1:divideIntoX
        temp(i,j) = SumOccupancy(map, zones(i + divideIntoY*(j-1),2), zones(i + divideIntoY*(j-1),1), divisionY, divisionX);
    end
end
temp
size(temp)
counter
holder = 0;
threshold = -360+25*counter;
posj = 0;
posi = 0;
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
regionToGo = [(divideIntoY-posj) posi-1]
% maxRegion = [(divideInto-posj+1)*divisionX ((posi+1)*divisionY -1)]
end

