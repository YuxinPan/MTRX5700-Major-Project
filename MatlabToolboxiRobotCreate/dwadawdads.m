clear all;
close all;
clc;

% Initialize communication
[serialObject] = RoombaInit(1)
% Read distance sensor (provides baseline)
InitDistance = DistanceSensorRoomba(serialObject);
%sets forward velocity 1 m/s and turning radius 2 m
% SetFwdVelAngVelCreate(serialObject, 0.1, 0 )
angle = pi/2;
x = 0;
y = 0;

figure(1);
hold on;
plot(x,y,'x');
axis equal;
grid on;

totalTime = 30;
dt = 0.25;
iterations = totalTime / dt;
p1 = plot(0,0,'rx');
SetFwdVelRadiusRoomba(serialObject, 0.2, 0.4);
%wait 1 second
for ticks = 1:iterations
   pause(dt);
   angle = angle + AngleSensorRoomba(serialObject);
   dist = DistanceSensorRoomba(serialObject);
   x = x + dist * cos(angle);
   y = y + dist * sin(angle);
   

    
   p2 = plot(x,y,'b.');
set(p1,'Visible','off')
   p1 = plot(x,y,'rx');
   drawnow;
   
   fprintf('T: %f angle: %f x: %f y: %f\n',ticks * 0.25, angle, x, y);
end

% stop the robot
SetFwdVelRadiusRoomba(serialObject, 0, inf);
% read the distance senor.
% returns dist since last reading in meters
Distance = DistanceSensorRoomba(serialObject)
