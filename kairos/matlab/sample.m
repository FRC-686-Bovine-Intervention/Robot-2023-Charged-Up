function [position, velocity, acceleration, jerk] = sample(time, totalTime, points)
    dt = totalTime / length(points);

    prev1 = floor(time/dt) +1;
    next1 = ceil(time/dt) +1;
    next1(next1==prev1) = next1(next1==prev1)+1;
    prev2 = prev1-1;
    next2 = next1+1;

    prev1 = clamp(prev1, 1, length(points));
    next1 = clamp(next1, 1, length(points));
    prev2 = clamp(prev2, 1, length(points));
    next2 = clamp(next2, 1, length(points));
    
    if size(points,2) > 1
        position(1,:) = interp1(linspace(0,totalTime,length(points)), points(1,:), time, 'linear');
        position(2,:) = interp1(linspace(0,totalTime,length(points)), points(2,:), time, 'linear');
    
        prevVelocity = (points(:,prev1) - points(:,prev2)) / dt;
        velocity = (points(:,next1) - points(:,prev1)) / dt;
        nextVelocity = (points(:,next2) - points(:,next1)) / dt;
    
        prevAcceleration = (velocity - prevVelocity) / dt;
        acceleration = (nextVelocity - velocity) / dt;
    
        jerk = (acceleration - prevAcceleration) / (2*dt);
    else
        position = points;
        velocity = zeros(size(position));
        acceleration = zeros(size(position));
        jerk = zeros(size(position));
end

