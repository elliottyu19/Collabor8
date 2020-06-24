function valid = checkSpeed(veltraj, maxVel)
    maxVels = max(veltraj, [], 2); %Find maximum velocities for each arm segment of robot
    maxofvel = max(maxVels); % Find the maximum velocity within all velocity values in the velocity profile
    if maxofvel > maxVel %Makes sure the maximum has not surpassed the maximum velocity allowed.
        valid = false; 
    else 
        valid = true;
    end
end