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
while 1
   %pause(dt);
   angle = angle + AngleSensorRoomba(serialObject);
   dist = DistanceSensorRoomba(serialObject);
   x = x + dist * cos(angle);
   y = y + dist * sin(angle);
   
   totalDistance = totalDistance+dist;
    
   F = [1 0 dist* sin(x_est(3));
        0 1 dist* cos(x_est(3));
        0 0 1];
   P = F*P*F' + Q;


    % perform a prediction
    x_est = [x_est(1) + dist* cos(x_est(3));
        x_est(2) + dist* sin(x_est(3));
        x_est(3) + angle];
   
   
   p2 = plot(x,y,'b.');
   set(p1,'Visible','off')
   p1 = plot(x,y,'rx');
   drawnow;
   
   fprintf('T: %f angle: %f x: %f y: %f\n',ticks * 0.25, angle, x, y);
   
   
   
   
   
   
   
    z = [x_est(1);
        x_est(2);
        headingObs];

    z_est = H*x_est;
    inn = z - z_est;

    if inn(3) > pi
        inn(3) = inn(3) - 2 * pi;
    elseif inn(3) < -pi
        inn(3) = inn(3) + 2 * pi;
    end

    inn_store = [inn_store inn];

    S = H*P*H' + R;
    S_store = [S_store S];

    W = P*H'*inv(S);

    x_est = x_est + W*inn;
    if x_est(3) > pi
        x_est(3) = x_est(3) - 2 * pi;
    elseif x_est(3) < -pi
        x_est(3) = x_est(3) + 2 * pi;
    end
    fprintf('Comp t: %d headingObs: %2.1f headingEst: %2.1f\n',time, headingObs, x_est(3));
    P = P - W*S*W';
    p1 = plot(x_est(1), x_est(2), 'g.');

   
   
   
   
   
   
   
   
   
   
   
   
   
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
