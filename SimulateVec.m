global trackParams;
global carParams;
global decisionVector;
global xInit;

main()
function main
global trackParams;
global carParams;
global decisionVector;
global xInit;
clf

[tout,qout] = simTrajectory(decisionVector); 
%[tout,qout] = simTrajectory(xInit); 

%DESPLAY TRACK ----------------------------------------------------
x1 = zeros(1,trackParams.trackResolution);
x2 = zeros(1,trackParams.trackResolution);
y1 = zeros(1,trackParams.trackResolution);
y2 = zeros(1,trackParams.trackResolution);

racingLine = [carParams.startPos(1), carParams.startPos(2), carParams.startSpeed]; %Initialize the racing Line to be Displayed

for i = 1:trackParams.trackResolution %Points of Track to Be Graphed
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
axis equal


%DRAW CAR ---------------------------------------------------------
hyp = sqrt(2 * carParams.size^2);
t1 = pi/4;
t2 = -pi/4;
t3 = 3 *pi/4;
t4 = -3 * pi/4;
    
xCar = [carParams.startPos(1) + hyp * sin(carParams.startTheta + t1), carParams.startPos(1) + hyp * sin(carParams.startTheta + t3), carParams.startPos(1) + hyp * sin(carParams.startTheta + t4),carParams.startPos(1) + hyp * sin(carParams.startTheta + t2)];
yCar = [carParams.startPos(2) + hyp * cos(carParams.startTheta + t1), carParams.startPos(2) + hyp * cos(carParams.startTheta + t3), carParams.startPos(2) + hyp * cos(carParams.startTheta + t4), carParams.startPos(2) + hyp * cos(carParams.startTheta + t2)];
car = fill(xCar,yCar,'g');
rLine = scatter(racingLine(:,1), racingLine(:,2), .0001);
%xlim([0 10])
%ylim([0 10])


%Use Path Resolution to Determine Number of Points for Racing Line
points = linspace(0, tout(end), trackParams.rLineResolution);

xPos = interp1(tout, qout(:,1), points);
xSpeed = interp1(tout, qout(:,2),points);
yPos = interp1(tout, qout(:,3),points);
ySpeed = interp1(tout, qout(:,4),points);
theta = interp1(tout, qout(:,5),points);


%ANIMATION FOR PATH -----------------------------------------------
for i = 1:trackParams.rLineResolution
   
    %Find New Cordinates for Drawing Car
    xCar = [xPos(i) + hyp * sin(theta(i) + t1), xPos(i) + hyp * sin(theta(i) + t3), xPos(i) + hyp * sin(theta(i) + t4), xPos(i) + hyp * sin(theta(i) + t2)];
    yCar = [yPos(i) + hyp * cos(theta(i) + t1), yPos(i) + hyp * cos(theta(i) + t3), yPos(i) + hyp * cos(theta(i) + t4), yPos(i) + hyp * cos(theta(i) + t2)];
    
    delete(car) %Delete and Draw New Car
    car = fill(xCar,yCar,'g');

    pause(.01)
    
    %Draw New Racing Line
    racingLine = [racingLine; [xPos(i), yPos(i), sqrt(xSpeed(i)^2 + ySpeed(i)^2)]];
    delete(rLine)
    rLine = scatter(racingLine(:,1), racingLine(:,2), 7, racingLine(:,3), 'filled');
    colormap cool;
    
    %Determine if Car is Within Track Limits
    pathPoint = closestPoint(trackParams.frenet, [xPos(i) yPos(i)]);
    pathPoint = pathPoint(1:2);
    dist = norm([xPos(i),yPos(i)] - pathPoint);

    if dist > trackParams.trackwidth
        set(gcf, 'Color', 'r');
    else
        set(gcf, 'Color', 'w');
    end
     
end

end
