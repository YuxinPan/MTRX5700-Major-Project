clear all;
close all;
clc;


warning('off','all') % don't want warning to stop whole system


resetFlag = 0;
s = urlread('http://mechatronics.top/demo/io.php?act=reset','Timeout',2.5);






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
    90/180*pi];
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




IMUserial = serial('COM3','BaudRate',115200)
fopen(IMUserial)
pause(10)
fprintf(IMUserial,'r');
text = fscanf(IMUserial)
headingOffset = str2num(extractAfter(text,"yaw:"))
headingOffset = (headingOffset-90)/180*pi



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
while 1
   %pause(dt);
   angleChange = AngleSensorRoomba(serialObject);
   angle = angle + angleChange;
   
   dist = DistanceSensorRoomba(serialObject);
   %x = x + dist * cos(angle);
   %y = y + dist * sin(angle);
   
   totalDistance = totalDistance+dist;
    
   F = [1 0 dist* sin(x_est(3));
        0 1 dist* cos(x_est(3));
        0 0 1];
   P = F*P*F' + Q;


    % perform a prediction
    x_est = [x_est(1) + dist* cos(x_est(3));
        x_est(2) + dist* sin(x_est(3));
        x_est(3) - angleChange];
   
   
   p2 = plot(x_est(1),x_est(2),'b.');
   drawnow;
   
   %fprintf('angleChange: %f x: %f y: %f\n', angleChange, x, y);
   
   
   
   % start polling heading
   fprintf(IMUserial,'r');
   text = fscanf(IMUserial);
   heading = str2num(extractAfter(text,"yaw:"));
   heading = heading/180*pi;
   
   
    z = [x_est(1);
        x_est(2);
        heading-headingOffset];

    
    if z(3) > pi
        z(3) = z(3) - 2 * pi;
    elseif z(3) < -pi
        z(3) = z(3) + 2 * pi;
    end
    
    z_est = H*x_est;
    inn = z - z_est;

    if inn(3) > pi
        inn(3) = inn(3) - 2 * pi;
    elseif inn(3) < -pi
        inn(3) = inn(3) + 2 * pi;
    end

    %inn_store = [inn_store inn];

    S = H*P*H' + R;
    %S_store = [S_store S];

    W = P*H'*inv(S);

    x_est = x_est + W*inn;
    if x_est(3) > pi
        x_est(3) = x_est(3) - 2 * pi;
    elseif x_est(3) < -pi
        x_est(3) = x_est(3) + 2 * pi;
    end
    fprintf('xEst: %2.1f yEst: %2.1f headingEst: %2.1f headingMag-offset: %2.1f angleChange: %2.f\n', x_est(1),x_est(2),x_est(3)*180/pi,(heading-headingOffset)*180/pi,angleChange*180/pi);
    P = P - W*S*W';
    %p1 = plot(x_est(1), x_est(2), 'g.');

   
   
   
   
   
   
   
   
   
   
   
   
   
   %if mod(ticks,3)==0
   s = urlread(strcat('http://mechatronics.top/demo/io.php?act=add&x=',num2str(x_est(1)),'&y=',num2str(x_est(2)),'&reset=',num2str(resetFlag)),'Timeout',2.5);
   %end
end
% stop the robot
SetFwdVelRadiusRoomba(serialObject, 0, inf);
% read the distance senor.
% returns dist since last reading in meters
%Distance = DistanceSensorRoomba(serialObject)
fprintf('Total Distance: ',totalDistance);