
clear all;
close all;
clc;

exist colorDevice;
if ans
    release(colorDevice);
end
exist depthDevice;
if ans
    release(depthDevice);
end

beep on;

%% Set up LIDAR SLAM
maxLidarRange = 7;
mapResolution = 20;
exist slamAlg;
if ~ans
    slamAlg = robotics.LidarSLAM(mapResolution, maxLidarRange);
end
slamAlg.LoopClosureThreshold = 100;
slamAlg.LoopClosureSearchRadius = 10;
slamAlg.LoopClosureAutoRollback = true;
% slamAlg.LoopClosureMaxAttempts = 0;

%% Set up distance to go
vel = 0.02 ;
distance = 4;
timeDuration = distance/vel;

turnRad = 0;


%% Lets connect to the devices
colorDevice = imaq.VideoDevice('kinect',1)
depthDevice = imaq.VideoDevice('kinect',2)




[serialObject] = RoombaInit(18)  % this is the serial port






scans = {};


%Initialise state
angle = pi/2;xx = 0; yy = 0;

% Run this forever
ii = 0;

% Start a timer to track runtime
tic;
figure('units','normalized','outerposition',[0 0 1 1]);

% Set current position as LIDAR 0,0,0
depthImage = step(depthDevice);
thisScan = getFakeLIDAR(depthImage, depthDevice);
[isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, thisScan, [0,0,pi/2]);
% Start moving
% SetFwdVelRadiusRoomba(serialObject, vel, turnRad);
t(1) = 0;
currentPose = [0,0,0];

% Goal total angle to turn(to start with, may change if can found)
goalXY = [];

inerObjectPos = [nan;nan];

while isempty(goalXY)
    while 1
        ii = ii + 1;

        %disp('here');

        %     travelDist(serialObject, 0.3, 0.2);
        turnAngle(serialObject, 0.1, 10);
        %disp('here2');

        % Get encoder data
        angleChange = AngleSensorRoomba(serialObject);
        angle = angle + angleChange;
        % Get the image LIDAR reading ASAP

        colorImage = step(colorDevice);
        depthImage = step(depthDevice);
        thisScan = getFakeLIDAR(depthImage,depthDevice);
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

        scans{end+1} = thisScan;

        % Add LIDAR scan to the model
        lidarPoseEst = [currentPose(1) + dist * cos(currentPose(3)+pi/2),...
            currentPose(2) + dist * sin(currentPose(3)+pi/2),...
            currentPose(3) + angleChange];
        %[isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scans{end},lidarPoseEst);
        [isScanAccepted, loopClosureInfo, optimizationInfo] = addScan(slamAlg, scans{end});
        %currentPose(3)
        %angle


        plot(thisScan.Cartesian(:,1), thisScan.Cartesian(:,2), 'b.');
        grid on;
        axis equal;
        axis([-5 5 0 8]);


        [scans, optimizedPoses]  = scansAndPoses(slamAlg);


        % Plot SLAM model and robot position
        subplot(1,3,3);
        title('SLAM Model and Vehicle Positions');
        show(slamAlg);
        %     SSetFwdVelRadiusRoomba(serPort, roombaSpeed, 0);
        currentPose = optimizedPoses(size(optimizedPoses,1),:);
        fprintf('Time stamp: %.2f\nLIDAR pose: %.3f,%.3f,%.3f\nLIDAR Pose est: %.3f,%.3f,%.3f\ndeadRec: %.3f,%.3f,%.3f\n',t(ii),currentPose(1),currentPose(2),currentPose(3),lidarPoseEst(1),lidarPoseEst(2),lidarPoseEst(3),xx,yy,angle-pi/2);
        %     vel = vel * -1;


        % detect can function
        [objectPos,canCount] = canDetection(colorImage,depthImage,depthDevice);
        
        if size(objectPos,2) > 0 && objectPos(2,1)<0.15 && objectPos(2,1)>-0.3
            % Transformation matrix to transform from body to inertial
            fprintf('can height: %.3f \n',objectPos(2,1));
            bod2inerM = [cos(currentPose(3)), sin(currentPose(3));
                -sin(currentPose(3)), cos(currentPose(3))]';

            inerObjectPos = bod2inerM * [objectPos(1,1)+0.2;objectPos(3,1)];
            inerObjectPos = inerObjectPos + [currentPose(1);currentPose(2)];
            hold on;
            exist p3;
            if ans
                delete(p3);
            end
            p3 = plot(inerObjectPos(1,:),inerObjectPos(2,:),'bx');  % this is the can detected
            if sum(isnan(inerObjectPos(:,1))) == 0
                goalXY = [inerObjectPos(1,1);inerObjectPos(2,1)];
            end
            fprintf('Can pos bdy: %.2f,%.2f,%.2f\n',objectPos(1,canCount),objectPos(2,canCount),objectPos(3,canCount));

        else
            hold on;
            p3 = plot(inerObjectPos(1,:),inerObjectPos(2,:),'bx'); % this is the can detected
        end

        exist p7;
        if ans
            delete(p7);
        end
        if ~isempty(goalXY)
            p7 = plot(goalXY(1),goalXY(2),'kx');
        end

        %% Clean up

        % Show the figure
        drawnow;

        % Lets do some time tracking
        t(ii+1) = toc;
        timeTaken(ii) = t(ii+1)-t(ii);

        % Stop running after however many seconds
        %     if t(ii+1) > timeDuration
        %         break;
        %     end
        if ii == 32
            break;
        end
    end
    if ~isempty(goalXY)
        break
    end
    travelDist(serialObject, 0.3, 0.2);
end    
    



% Stop moving
SetFwdVelRadiusRoomba(serialObject, 0, inf);
release(colorDevice);
release(depthDevice);
fprintf('Script ended with %.0f scans after %.2f seconds, average time per scan: %.2f seconds\n',ii, t(ii), t(ii)/ii);
figure();
plot(timeTaken);
title('Time taken for each loop');

[scans, optimizedPoses]  = scansAndPoses(slamAlg);
map = buildMap(scans, optimizedPoses, mapResolution, maxLidarRange);

figure();
subplot(1,2,1);
show(map);
hold on
show(slamAlg.PoseGraph, 'IDs', 'off');
hold off
title('Occupancy Grid Map Built Using Lidar SLAM');
%% path planning

%if ~isempty(inerObjectPos) % if there is a can, then go for it
subplot(1,2,2);
myOccupancyShow=pathPlan(map,goalXY);
imagesc(myOccupancyShow);
%end

figure();


goalAngle = -atan2(goalXY(1),goalXY(2));
goalXY
goalAngle
currentPose(3)
rad2deg(goalAngle-currentPose(3))
turnAngle(serialObject, 0.1, rad2deg(goalAngle-currentPose(3)));
angle = 0;
dist=0;
currentPose=[0;0;0];
ii=0;
while 1
    ii = ii + 1;

    % Get encoder data
    angleChange = AngleSensorRoomba(serialObject);
    angle = angle + angleChange;
    % Get the image LIDAR reading ASAP

    [dist] = DistanceSensorRoomba(serialObject);
%     xx = xx + dist * cos(angle);
%     yy = yy + dist * sin(angle);
% 
%     scans{end+1} = thisScan;

    % Add LIDAR scan to the model
    currentPose(1) = currentPose(1) + dist * cos(currentPose(3)+pi/2);
    currentPose(2) = currentPose(2) + dist * sin(currentPose(3)+pi/2);
    currentPose(3) = currentPose(3) + angleChange;
    
    %% Try find the can
    [objectPos,canCount] = canDetection(colorImage,depthImage,depthDevice);

    objectPos
    
    if size(objectPos,2) > 0 && objectPos(2,1)<0.15 && objectPos(2,1)>-0.3
        % Transformation matrix to transform from body to inertial
        bod2inerM = [cos(currentPose(3)), sin(currentPose(3));
            -sin(currentPose(3)), cos(currentPose(3))]';
        
        inerObjectPos = bod2inerM * [objectPos(1,1)+0.2;objectPos(3,1)];
        inerObjectPos = inerObjectPos + [currentPose(1);currentPose(2)];
        hold on;
        exist p3;
        if ans
            delete(p3);
        end
        p3 = plot(inerObjectPos(1,:),inerObjectPos(2,:),'bx');  % this is the can detected
        if sum(isnan(inerObjectPos(:,1))) == 0
            
            goalXY = [inerObjectPos(1,1);inerObjectPos(2,1)];
        end
        fprintf('Can pos bdy: %.2f,%.2f,%.2f\n',objectPos(1,canCount),objectPos(2,canCount),objectPos(3,canCount));

    else
        hold on;
        p3 = plot(inerObjectPos(1,:),inerObjectPos(2,:),'bx'); % this is the can detected
    end
    
        %% Control movement
    if norm(goalXY) < 1
        turnAngle(serialObject, 0.1, 7);
    else
        goalAngle = -atand(goalXY(1)/goalXY(2));
        angleDiff = goalAngle - rad2deg(currentPose(3));
        if angleDiff > 7
            turnAngle(serialObject, 0.1, 7);
        elseif angleDiff < -7
            turnAngle(serialObject, 0.1, -7);
        else % If we're within the angle goal
            distToTravel = norm([goalXY(1)-currentPose(1),goalXY(2)-currentPose(2)]);
            if distToTravel > 0.1
                travelDist(serialObject, 0.4, 0.4);
            end
        end
    end

    hold on;
%     exist p4;
%     if ans
%         delete(p4);
%     end
%     if ~isempty(goalXY)
%         p4 = plot(goalXY(1),goalXY(2),'kx');
%     end
    p4 = plot(currentPose(1),currentPose(2),'bx');
    %% Clean up
    
    % Show the figure
    drawnow;
    
    % Lets do some time tracking
    t(ii+1) = toc;
    timeTaken(ii) = t(ii+1)-t(ii);
    
    % Stop running after however many seconds
    %     if t(ii+1) > timeDuration
    %         break;
    %     end
    if ii == 40 
        break;
    end
end

%%




function scanData = getFakeLIDAR(depthImage, depthDeviceF)

% Downsampling grid size / m
gridAve = 0.10;

beep;


ptCloud = pcfromkinect(depthDeviceF,depthImage);
ptCloud = pcdenoise(ptCloud);
ptCloud = pcdownsample(ptCloud,'gridAverage',gridAve);
% ptCloudOut = pcdownsample(ptCloud,'nonuniformGridSample',80);

%     x = reshape(ptCloudOut.Location(:,:,1),[],1)';
%     y = reshape(ptCloudOut.Location(:,:,2),[],1)';
%     z = reshape(ptCloudOut.Location(:,:,3),[],1)';

xyz = ptCloud.Location;
k = find(xyz(:,2) > -0.3);
xyz = xyz(k,:);
k = find(xyz(:,2) < gridAve - 0.3);
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


function [objectPos,canCount] = canDetection(colorImage,depthImage,depthDevice)
    %% Try find the can
    I_hsv = rgb2hsv(colorImage);

    % Red sits around zero, but wraps to one.
    % I_hsv should be float values in the range [0.0, 1.0]
    lowBound = 0.98;
    highBound = 0.02;

    I_hue = I_hsv(:,:,1);
    I_sat = I_hsv(:,:,2);
    I_val = I_hsv(:,:,3);

    I_red = (I_hue > lowBound) | (I_hue < highBound);
    I_red = I_red & (I_sat>0.5) & (I_val > 0.6);
    se = strel('disk',40);
    cansBinaryImage = imclose(I_red,se);

    stats = [regionprops(cansBinaryImage,'Centroid', 'area', 'BoundingBox', 'PixelIdxList')];

    % Increment for creating an array of can positions
    canCount = 0;
    objectPos = [];
    for stat = 1:length(stats)
        if stats(stat).Area > 100
            % Increment counter
            canCount = canCount + 1;

            % Get a fresh clear image
            colorImage1=uint8(ones(1080,1920,3).*255);

            % Plot red outlines of cans
            %             rectangle('Position', stats(stat).BoundingBox,'Linewidth', 3, 'EdgeColor', 'r', 'LineStyle', '--');

            % Change colorImage1 pixels that are part of the can from white to
            % black(255 to 0)
            temp = colorImage1(:,:,1);temp(stats(stat).PixelIdxList) = 0;colorImage1(:,:,1) = temp;
            temp = colorImage1(:,:,2);temp(stats(stat).PixelIdxList) = 0;colorImage1(:,:,2) = temp;
            temp = colorImage1(:,:,3);temp(stats(stat).PixelIdxList) = 0;colorImage1(:,:,3) = temp;
%             parfor n=1:3
%                temp = colorImage1(:,:,n);temp(stats(stat).PixelIdxList) = 0;colorImage1(:,:,n) = temp;
%             end
            % Feed the binary image into a point cloud function
            % to let the function
            % figure out the x, y, and z positions.
            ptCloud = pcfromkinect(depthDevice,depthImage,colorImage1);


            % Isolate the points that are part of the can(by finding colours
            % that = 0), then take the average of their x, y, and z positions
            k=find(ptCloud.Color(:,:,1)==0);
            %parfor n=1:3
            temp=ptCloud.Location(:,:,1);temp=temp(k);temp(isnan(temp)) = []; objectPos(1,canCount) = mean(temp);
            temp=ptCloud.Location(:,:,2);temp=temp(k);temp(isnan(temp)) = []; objectPos(2,canCount) = mean(temp);
            temp=ptCloud.Location(:,:,3);temp=temp(k);temp(isnan(temp)) = []; objectPos(3,canCount) = mean(temp);
            %    temp=ptCloud.Location(:,:,n);temp=temp(k);temp(isnan(temp)) = []; objectPos(n,canCount) = mean(temp);
            %end
            % Display text next to the cans to show us their positions
            %             text(round(stats(stat).Centroid(1))+30,round(stats(stat).Centroid(2))+20, sprintf('x: %.2fm, y: %.2fm, z: %.2fm',objectPos(1,canCount),objectPos(2,canCount),objectPos(3,canCount)));
        end
    end
end