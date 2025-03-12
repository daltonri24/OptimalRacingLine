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
clf
%GLOBAL PARAMS
dT = .25;
N = 10;

%CAR PARAMS
carParams.startSpeed = 0;
carParams.startTheta = pi/2;
%carParams.startTheta = trackParams.states(1,3);
carParams.startPos = trackParams.states(1,1:2);
carParams.size = .05;
carParams.dragCoef = .5;

%Inputs
acceleration = 0;
steer = 0;

%Current Car State
accXY = [0,0];
speedXY = [cos(carParams.startTheta) * carParams.startSpeed, sin(carParams.startTheta) * carParams.startSpeed];
dir = carParams.startTheta;
pos = carParams.startPos;

%Values For Decision Vector
accStore = [acceleration];
steerStore = [steer];
time = 0;

racingLine = [pos(1), pos(2), sqrt(speedXY(1)^2 + speedXY(2)^2)];


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

%Draw Initial Car
hyp = sqrt(2 * carParams.size^2);
t1 = pi/4;
t2 = -pi/4;
t3 = 3 *pi/4;
t4 = -3 * pi/4;
x = [pos(1) + hyp * sin(dir + t1), pos(1) + hyp * sin(dir + t3), pos(1) + hyp * sin(dir + t4),pos(1) + hyp * sin(dir + t2)];
y = [pos(2) + hyp * cos(dir + t1), pos(2) + hyp * cos(dir + t3), pos(2) + hyp * cos(dir + t4), pos(2) + hyp * cos(dir + t2)];
car = fill(x,y,'g');

%Draw Inital Racing Line
rLine = scatter(racingLine(:,1), racingLine(:,2));

%Draw Intial Readings
accText = text(0,0,string(acceleration));
speedText = text(0,1,append('Speed:', string(0)));
steerText = text(0,2,append('Steer:', string(steer)));

%User Input Handling
ax = gcf;
set(ax,'KeyPressFcn',@func); 
    function func(~,event) 
        
        if strcmp(event.Key, 'rightarrow')
            if(steer > -pi/12)
                steer = steer - .1;
            end
        elseif strcmp(event.Key, 'leftarrow')
            if(steer < pi/12)
                steer = steer + .1;
            end
        elseif strcmp(event.Key, 'uparrow')
            acceleration = acceleration + .0035;
        elseif strcmp(event.Key, 'downarrow')
            acceleration = acceleration - .0035;
        end
    end


while true
 
    %Draw Car
    x = [pos(1) + hyp * sin(dir + t1), pos(1) + hyp * sin(dir + t3), pos(1) + hyp * sin(dir + t4),pos(1) + hyp * sin(dir + t2)];
    y = [pos(2) + hyp * cos(dir + t1), pos(2) + hyp * cos(dir + t3), pos(2) + hyp * cos(dir + t4), pos(2) + hyp * cos(dir + t2)]; 
    delete(car)
    car = fill(x,y,'g');
    
    %Simluated Movement of Car
    pause(.05)
    
    speed = sqrt(speedXY(1)^2 + speedXY(2)^2);
    dirChange = ( speed / carParams.size ) * tan(steer);
    
    pos(1) = pos(1) + speedXY(1) * dT;
    pos(2) = pos(2) + speedXY(2) * dT;
    
    oldSpeedXY = speedXY;
    
    speedXY(1) = speedXY(1) + accXY(1) * dT;
    speedXY(2) = speedXY(2) + accXY(2) * dT;
    
    accXY(1) = (speed * -sin(dir) * dirChange) + (acceleration * cos(dir)) - sign(oldSpeedXY(1)) * carParams.dragCoef * oldSpeedXY(1)^2;
    accXY(2) = (speed * cos(dir) * dirChange) + (acceleration * sin(dir)) - sign(oldSpeedXY(2)) * carParams.dragCoef * oldSpeedXY(2)^2;
    
    dir = dir + dirChange * dT;
    
    %Store Both the Acceleration and Steering for Creating Drive Path/Decision Vector
    accStore = [accStore, acceleration];
    steerStore = [steerStore, steer];
    time = time + dT;     %Interp Into Correct Sized Decision Vector/X0
    newAcc = interp1(1:length(accStore),accStore,linspace(1,length(accStore),N));
    newSteer = interp1(1:length(steerStore),steerStore,linspace(1,length(steerStore),N));
    decisionVector = [newAcc, newSteer, time];
    xInit = [newAcc, newSteer, time];
    
    %Create the Racing Line to Display with Gradient Color for Speed
    racingLine = [racingLine; [pos(1), pos(2), sqrt(speedXY(1)^2 + speedXY(2)^2)]];
    delete(rLine)
    rLine = scatter(racingLine(:,1), racingLine(:,2), 7, racingLine(:,3), 'filled'); 
    delete(accText)
    accText = text(0,0,append('Acc:', string(acceleration)));
    delete(speedText)
    speedText = text(0,.5,append('Speed:', string(speed)));
    delete(steerText)
    steerText = text(0,1,append('Steer:', string(steer)));
    colormap cool;
    
    
    %Determine the Distance of Car from Track
    pathPoint = closestPoint(trackParams.frenet, [pos(1) pos(2)]);
    pathPoint = pathPoint(1:2);

    dist = norm([pos(1),pos(2)] - pathPoint);

    if dist > trackParams.trackwidth
        set(ax, 'Color', 'r');
    else
        set(ax, 'Color', 'w');
    end
    
    %Normalize Steering
    if steer > 0.049
        steer = steer - .05;
    elseif steer < -0.049
        steer = steer + .05;
    end
    
end

end