function [tout,yout] = simTrajectory(x)
    global carParams;
   
    N = (length(x)-1)/2;

    T = x(end);
    acceleration = x(1:N);
    steer = x(N+1:2*N);

    interp_acc = @(t) interp1(linspace(0,T,N), acceleration,t);
    interp_steer = @(t) interp1(linspace(0,T,N), steer, t);
           
        function res = odefun(t,y)
            speed = sqrt(y(2)^2 + y(4)^2);

            dTheta = ( speed / carParams.size ) * tan(interp_steer(t));
            dx = y(2);
            ddx = -speed * sin(y(5)) * dTheta + interp_acc(t) * cos(y(5)) - sign(y(2)) * carParams.dragCoef * y(2)^2;
            dy = y(4);
            ddy = speed * cos(y(5)) * dTheta + interp_acc(t) * sin(y(5)) - sign(y(4)) * carParams.dragCoef * y(4)^2;

            res = [dx; ddx; dy; ddy; dTheta];
        end
        
    [tout, yout] = ode45(@odefun, linspace(0,T,500), [carParams.startPos(1); carParams.startSpeed*cos(carParams.startTheta); carParams.startPos(2); carParams.startSpeed*sin(carParams.startTheta); carParams.startTheta]); 
    
    %Displays Current Progress of Optimization, Can Be Commented Out
    if(rand > .0)
        plotPath(tout, yout)
    end
end


function plotPath(tout, qout)
    global trackParams
    
    clf 
    
    x1 = zeros(1,trackParams.trackResolution);
    x2 = zeros(1,trackParams.trackResolution);
    y1 = zeros(1,trackParams.trackResolution);
    y2 = zeros(1,trackParams.trackResolution);
    for i = 1:trackParams.trackResolution
        x1(i) = trackParams.trackwidth * cos(trackParams.states(i,3) + pi/2) + trackParams.states(i,1);
        x2(i) = trackParams.trackwidth * cos(trackParams.states(i,3) - pi/2) + trackParams.states(i,1);
        y1(i) = trackParams.trackwidth * sin(trackParams.states(i,3) + pi/2) + trackParams.states(i,2);
        y2(i) = trackParams.trackwidth * sin(trackParams.states(i,3) - pi/2) + trackParams.states(i,2);
    end

    clf
    hold on
    plot(x1, y1, 'b');
    plot(x2, y2, 'b');
    plot([x1(1), x2(1)], [y1(1), y2(1)], 'r');
    
    points = linspace(0, tout(end), trackParams.rLineResolution);
    
    xPos = interp1(tout, qout(:,1), points);
    xSpeed = interp1(tout, qout(:,2),points);
    yPos = interp1(tout, qout(:,3),points);
    ySpeed = interp1(tout, qout(:,4),points);
    
    scatter(xPos, yPos, 7, sqrt(ySpeed.^2 + xSpeed.^2), 'filled');
    
    check = nonlconCheck(tout, qout);
    if(check == 0)
        set(gcf, 'Color', 'r');
    else
        set(gcf, 'Color', 'w');
    end
    pause(.0001)
end

function pass = nonlconCheck(tout, qout)
    global trackParams;
    global carParams;

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
   
    pass = 0;
    if(sum(C > 0) == 0 && sum(Ceq == 0) == 0)
        pass = 1;
    end
end