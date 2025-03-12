global trackParams;
global carParams;
global decisionVector;
global xInit;
global N;

main()

function main
global trackParams;
global carParams;

global decisionVector;
global xInit;
global N;

x0 = [1*ones(N * 2,1);  30];
%x0 = xInit;

options = optimoptions('fmincon','MaxFunctionEvaluations', 100000, 'DiffMinChange', .0025, 'StepTolerance', 1e-8, 'FiniteDifferenceType', 'central');
%options = optimoptions('fmincon','MaxFunctionEvaluations', 100000);
xstar = fmincon(@costfcn, x0, [], [], [], [], [-.7*ones(N,1); -pi/10*ones(N,1); 0], [.7*ones(N,1); pi/10*ones(N,1); Inf], @nonlcon, options);

decisionVector = xstar;

end


function cost = costfcn(x)
global trackParams; 

[tout, qout] = simTrajectory(x);

endCost = max(sqrt( (trackParams.states(end,1) - qout(end,1))^2 + (trackParams.states(end,2) - qout(end,3))^2) - trackParams.trackwidth, 0);

xyCords = [qout(:,1), qout(:,3)];
points = closestPoint(trackParams.frenet, xyCords);
points = points(:,1:2);
pathCost = sum(max(vecnorm(xyCords - points, 2, 2) - trackParams.trackwidth, 0)) / (length(qout(:,1)) * trackParams.trackwidth);

lastPathPoint = closestPoint(trackParams.frenet, trackParams.frenet.Waypoints(end-1,:));
pathLength = lastPathPoint(6);
distanceTraveled = sum(sqrt((xyCords(2:end,1) - xyCords(1:end-1,1)).^2 + (xyCords(2:end,2) - xyCords(1:end-1,2)).^2));
lengthCost = abs(1 - abs(distanceTraveled / pathLength));
minLengthCost = abs(distanceTraveled / pathLength);
if lengthCost < 1
    lengthCost = 0;
else
    lengthCost = lengthCost - 1;
end

if minLengthCost < .3
    minLengthCost = 1000;
else
    minLengthCost = 0;
end

cost = tout(end) + endCost * (tout(end) / .05) + pathCost * (tout(end) / .05) + lengthCost * (tout(end) / .1) + minLengthCost * (tout(end) / .001)
%cost = tout(end) + pathCost * (tout(end) / .05) + lengthCost * (tout(end) / .1) + minLengthCost * (tout(end) / .001)

end