function locationArray = FindNextLocation(map, counter)
%FINDNEXTLOCATION Summary of this function goes here
%   Detailed explanation goes here
    headTo = 1;
    headToArray = zeros(3,2);    
    xsize = 26.0;
    ysize = 21.0;
    mapRes = 20.0;
    xRan = 0;
    yRan = 0;
    cords = DivideMap(map, 21, 26, 20, counter)
    for j = 1:3
        countingCounter = 1
        while (headTo ~= 0)
            xRan = (cords(2)+rand(1));
            yRan = (cords(1)+rand(1));
            headTo = checkOccupancy(map,[xRan yRan]);
            countingCounter = countingCounter + 1;
            if (countingCounter > 50)
                break
            end
        end
        if (countingCounter > 50)
            headToArray = zeros(3,2);
            break
        end
        headTo = 1;
        headToArray(j,:) = [xRan yRan];
    end
    locationArray = headToArray
    

end

