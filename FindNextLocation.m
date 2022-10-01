function locationArray = FindNextLocation(map, counter)
%   This function finds the next location on the map for Robi the robot to
%   go to
    
    % Initilization
    headTo = 1;
    headToArray = zeros(3,2);
    
    % The map size
    xsize = 26.0;
    ysize = 21.0;
    mapRes = 20.0;
    xRan = 0;
    yRan = 0;
    
    % Divides the map into squares and gets the cords of the next square to
    % check out
    cords = DivideMap(map, 21, 26, 20, counter)
    
    % Simply guesses three times the next location on the map to go to by
    % using random guesses
    for j = 1:3
        countingCounter = 1
        while (headTo ~= 0)
            xRan = (cords(2)+rand(1));
            yRan = (cords(1)+rand(1));
            headTo = checkOccupancy(map,[xRan yRan]);       % Only procceed if it is unoccupied
            countingCounter = countingCounter + 1;
            if (countingCounter > 50)
                break
            end
        end
        % Attempt to not get stuck in a loop if it cannot find a location
        % to go to
        if (countingCounter > 50)
            headToArray = zeros(3,2);
            break
        end
        headTo = 1;
        headToArray(j,:) = [xRan yRan];
    end
    % The location to head to
    locationArray = headToArray
    

end

