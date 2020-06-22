function valid = checkSpeed(veltraj, maxVel)
    maxVels = max(veltraj, [], 2);
    maxofvel = max(maxVels);
    if maxofvel > maxVel
        valid = false;
    else 
        valid = true;
    end
end