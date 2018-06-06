clear all;
close all;
clc;

exist colorDevice;
if ans
    release(colorDevice);
    release(depthDevice);
end

beep on;

%% Set up LIDAR SLAM
maxLidarRange = 7;
mapResolution = 30;
slamAlg = robotics.LidarSLAM(mapResolution, maxLidarRange);
slamAlg.LoopClosureThreshold = 100;
slamAlg.LoopClosureSearchRadius = 1;
slamAlg.LoopClosureAutoRollback = false;
slanAlg.LoopClosureMaxAttempts = 0;




%% Lets connect to the devices
colorDevice = imaq.VideoDevice('kinect',1)
depthDevice = imaq.VideoDevice('kinect',2)
[serialObject] = RoombaInit(3)  % this is the serial port
scans = {};
vel = 0.2 ;
SetFwdVelRadiusRoomba(serialObject, vel, inf);

%Initialise state
angle = pi/2;xx = 0; yy = 0;

% Run this forever
ii = 0;
tic;
while 1
    ii = ii + 1;
    figure(1);
    
    
    % Get encoder data
    angleChange = AngleSensorRoomba(serialObject);
    angle = angle + angleChange;
    [dist] = DistanceSensorRoomba(serialObject);
    xx = xx + dist * cos(angle);
    yy = yy + dist * sin(angle);
    
    % Plot dead reckoning
    subplot(1,3,1);
    hold on;
    grid on;
    axis equal;
    plot(xx,yy,'b.');
    
    
    
    
    % Plot LIDAR data
    subplot(1,3,2);
    thisScan = getFakeLIDAR(depthDevice);
    scans{end+1} = thisScan;
    
    
    [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scans{end});
    t(ii) = toc;
    
    
    plot(thisScan.Cartesian(:,1), thisScan.Cartesian(:,2), 'b.');
    grid on;
    axis equal;
    axis([-5 5 0 8]);
    
    
    [scans, optimizedPoses]  = scansAndPoses(slamAlg);
    
    
    % Plot SLAM model and robot position
    subplot(1,3,3);
    show(slamAlg);
    %     SSetFwdVelRadiusRoomba(serPort, roombaSpeed, 0);
    currentPose = optimizedPoses(size(optimizedPoses,1),:);
    fprintf('Time stamp: %d LIDAR pose: %f,%f,%f deadRec: %f,%f,%f\n',t(ii),currentPose(1),currentPose(2),currentPose(3),xx,yy,angle);
    %     vel = vel * -1;
    
    % Show the figure
    drawnow;
    
    
    % Stop running after however many seconds
    if t(ii) > 10
        break;
    end
end
% Stop moving
SetFwdVelRadiusRoomba(serialObject, 0, inf);
release(colorDevice);
release(depthDevice);
disp('Script ended');








function scanData = getFakeLIDAR(depthDeviceF)
gridAve = 0.2;

beep;
depthImage = step(depthDeviceF);

ptCloud = pcfromkinect(depthDeviceF,depthImage);
ptCloudOut = pcdownsample(ptCloud,'gridAverage',gridAve);

%     x = reshape(ptCloudOut.Location(:,:,1),[],1)';
%     y = reshape(ptCloudOut.Location(:,:,2),[],1)';
%     z = reshape(ptCloudOut.Location(:,:,3),[],1)';

xyz = ptCloudOut.Location;
k = find(xyz(:,2) > -0.4);
xyz = xyz(k,:);
k = find(xyz(:,2) < gridAve - 0.4);
xyz = xyz(k,:);
fprintf('Point count points: %d\n',size(xyz,1));
%     view(player,ptCloud);

%     plot(xyz(:,1), xyz(:,3),'.');
%     xlabel('x');
%     ylabel('z');
%     %    zlabel('z');
%     axis equal;
%     axis([-5 5 0 9]);
%     grid on;
%     drawnow;
%
%     fprintf('Picture: %d\n',i);
scanData = lidarScan(double([xyz(:,1),xyz(:,3)]));
%     pause(0.5);

end