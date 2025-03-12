function [C, Ceq] = nonlcon(x)
    global trackParams;
    global carParams;
    
    [tout, qout] = simTrajectory(x);
    %---------------------------------------------------
    %Contraint For Car In Track Bounds
    xyCords = [qout(:,1), qout(:,3)];
    points = closestPoint(trackParams.frenet, xyCords);
    points = points(:,1:2);
    distConstraint = vecnorm(xyCords - points, 2, 2) - trackParams.trackwidth;

    speed = vecnorm([qout(:,2), qout(:,4)], 2, 2);
    
    distEnd = sqrt( (trackParams.states(end,1) - qout(end,1))^2 + (trackParams.states(end,2) - qout(end,3))^2 );

    C = [distConstraint', -speed', distEnd - trackParams.trackwidth];
    Ceq = [];
    %---------------------------------------------------
end