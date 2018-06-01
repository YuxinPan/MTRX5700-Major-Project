clear all;
close all;
clc;

resetFlag = 0;
s = urlread('http://mechatronics.top/demo/io.php?act=reset');






hold on
grid on
axis equal

% Sort it according to time

% actual states
% x = [positionObs(1,3); positionObs(1,4)]

% estimator states
% Define our state as:
% [x;y;psi]
% initUncertainty = [0.1; 0.5];
x_est = [0;
    0;
    0];
P = zeros(3,3);
% P(1,1) = initUncertainty(1,1)^2;
% P(2,2) = initUncertainty(2,1)^2;

% process model
% F = [1 dt; 0 1];
% v = [0.0; 0.01];
% Uncertainty in model state
Q = zeros(3,3);
Q(1,1) = 0.01^2;
Q(2,2) = 0.01^2;
Q(3,3) = 0.02^2;

% observation model
H = [1 0 0;
    0 1 0;
    0 0 1];

% Uncertainty in observations
R = [0.5^2 0 0;
    0 0.5^2 0;
    0 0 0.25^2];%m











% Initialize communication
[serialObject] = RoombaInit(18)  % this is the serial port
% Read distance sensor (provides baseline)
InitDistance = DistanceSensorRoomba(serialObject);
%sets forward velocity 1 m/s and turning radius 2 m
% SetFwdVelAngVelCreate(serialObject, 0.1, 0 )
angle = pi/2;
x = 0;
y = 0;

totalDistance = 0;

figure(1);
hold on;
plot(x,y,'x');
axis equal;
grid on;

totalTime = 20;
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
   
   totalDistance = totalDistance+dist;
    
   p2 = plot(x,y,'b.');
   set(p1,'Visible','off')
   p1 = plot(x,y,'rx');
   drawnow;
   
   fprintf('T: %f angle: %f x: %f y: %f\n',ticks * 0.25, angle, x, y);
   if mod(ticks,3)==0
       s = urlread(strcat('http://mechatronics.top/demo/io.php?act=add&x=',num2str(x),'&y=',num2str(y),'&reset=',num2str(resetFlag)));
   end
end
% stop the robot
SetFwdVelRadiusRoomba(serialObject, 0, inf);
% read the distance senor.
% returns dist since last reading in meters
%Distance = DistanceSensorRoomba(serialObject)
fprintf('Total Distance: ',totalDistance);
