global trackParams;

main()

function main
global trackParams;
clf

trackParams.trackResolution = 500; %Resolution of the Track
trackParams.rLineResolution = 500; %Resolution of the Racing Line Produced by the Car In Simulations
trackParams.trackwidth = .2; %Track Width
trackParams.states = []; %Track Representation
trackParams.frenet = [];

waypoints = [...
    1 1;
    1 1.2;
];

trackParams.frenet = referencePathFrenet( [waypoints; waypoints(1,:)] );
trackParams.states = createPath(trackParams.frenet, trackParams.trackResolution);

grid on;
axis equal;
hold on;
plot(trackParams.states(:,1), trackParams.states(:,2));
plot(waypoints(:,1), waypoints(:,2), ".r", "MarkerSize", 10)
xlim([0 10])
ylim([0 10])

ax = gca;
set(ax,'buttondownfcn',@func); % assign function to gca
    function func(~,~)        
        x = ax.CurrentPoint(1,1);
        y = ax.CurrentPoint(1,2);
           
        waypoints = [waypoints; [x y]];% get coordinates of click
        
        trackParams.frenet = referencePathFrenet([waypoints; waypoints(1,:)]);
        trackParams.states = createPath(trackParams.frenet, trackParams.trackResolution);
        
        cla(ax)
        plot(trackParams.states(:,1), trackParams.states(:,2));
        plot(waypoints(:,1), waypoints(:,2), ".r", "MarkerSize", 10)
    end
end

function states = createPath(frenet,numPoints)

lastPathPoint = closestPoint(frenet, frenet.Waypoints(end-1,:));
pathLength = lastPathPoint(6);

arclengths = linspace(0, pathLength+10, numPoints);

states = interpolate(frenet, arclengths);
states = states(:, 1:3);
end
