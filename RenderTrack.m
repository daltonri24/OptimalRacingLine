global trackParams;

main()

function main
global trackParams;
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

hold on
plot(x1, y1, 'b');
plot(x2, y2, 'b');

ax = gca;
set(ax,'buttondownfcn',@func); 
function func(~,~)        
        x = ax.CurrentPoint(1,1);
        y = ax.CurrentPoint(1,2);
           
        plot(x,y, '.g')
        
        pathPoint = closestPoint(trackParams.frenet, [x y]);
        pathPoint = pathPoint(1:2);
        
        dist = norm([x,y] - pathPoint);
        
        if dist > trackParams.trackwidth
            set(ax, 'Color', 'r');
        else
            set(ax, 'Color', 'w');
        end
    end

end
