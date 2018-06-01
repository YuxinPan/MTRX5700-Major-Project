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
maxLidarRange = 8;
mapResolution = 30;
slamAlg = robotics.LidarSLAM(mapResolution, maxLidarRange);
slamAlg.LoopClosureThreshold = 210;  
slamAlg.LoopClosureSearchRadius = 8; 




%% Lets connect to the devices
colorDevice = imaq.VideoDevice('kinect',1)
depthDevice = imaq.VideoDevice('kinect',2)
[serialObject] = RoombaInit(3)  % this is the serial port
scans = {};
vel = 0.025 ;
SetFwdVelRadiusRoomba(serialObject, vel, 0);
while 1
    figure(1);
    subplot(1,2,1);
    thisScan = getFakeLIDAR(depthDevice);
    scans{end+1} = thisScan;
    [isScanAccepted, l oopClosureInfo, optimizationInfo] = addScan(slamAlg, scans{end});
    subplot(1,2,2);
    show(slamAlg);
%     SSetFwdVelRadiusRoomba(serPort, roombaSpeed, 0);
    vel = vel * -1;
end










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
    fprintf('Point count: %d\n',size(xyz,1));
%     view(player,ptCloud);

    plot(xyz(:,1), xyz(:,3),'.');
    xlabel('x');
    ylabel('z');
    %    zlabel('z');
    axis equal;
    axis([-5 5 0 9]);
    grid on;
    drawnow;
    
%     fprintf('Picture: %d\n',i);
    scanData = lidarScan(double([xyz(:,1),xyz(:,3)]));
%     pause(0.5);

end