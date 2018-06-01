%% Implement Simultaneous Localization And Mapping (SLAM) with Lidar Scans 
%% Introduction
close all;
clear all;
clc;

gridAve = 0.2;
% This example demonstrates how to implement the Simultaneous Localization
% And Mapping (SLAM) algorithm on a collected series of lidar scans using
% pose graph optimization. The goal of this example is to build a map of
% the environment using the lidar scans and retrieve the trajectory of the
% robot.
%
% To build the map of the environment, the SLAM algorithm incrementally
% processes the lidar scans and builds a pose graph that links these scans.
% The robot recognizes a previously-visited place through scan matching and
% may establish one or more loop closures along its moving path. The SLAM 
% algorithm utilizes the loop closure information to update the map 
% and adjust the estimated robot trajectory.

% Copyright 2017 The MathWorks, Inc.

%% Load Laser Scan Data from File
% Load a down-sampled data set consisting of laser scans collected from a
% mobile robot in an indoor environment. The average displacement between
% every two scans is around 0.6 meters.

%%
%% If we haven't released the devices, then release them
exist colorDevice;
if ans
    release(colorDevice);
    release(depthDevice);
end


%% Lets connect to the devices
colorDevice = imaq.VideoDevice('kinect',1)
depthDevice = imaq.VideoDevice('kinect',2)

colorImage = step(colorDevice);
depthImage = step(depthDevice);

ptCloud = pcfromkinect(depthDevice,depthImage,colorImage);

% player = pcplayer(ptCloud.XLimits,ptCloud.YLimits,ptCloud.ZLimits,...
%     'VerticalAxis','y','VerticalAxisDir','down');
% 
% xlabel(player.Axes,'X (m)');
% ylabel(player.Axes,'Y (m)');
% zlabel(player.Axes,'Z (m)');

% Show 500 images
    figure(1);
    hold off;

    beep on;
for i = 1:40
    beep;
    colorImage = step(colorDevice);
    depthImage = step(depthDevice);
    
    ptCloud = pcfromkinect(depthDevice,depthImage);
    ptCloudOut = pcdownsample(ptCloud,'gridAverage',gridAve);
    
%     x = reshape(ptCloudOut.Location(:,:,1),[],1)';
%     y = reshape(ptCloudOut.Location(:,:,2),[],1)';
%     z = reshape(ptCloudOut.Location(:,:,3),[],1)';
    
    xyz = ptCloudOut.Location;
    k = find(xyz(:,2) > 0);
    xyz = xyz(k,:);
    k = find(xyz(:,2) < gridAve);
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
    
    fprintf('Picture: %d\n',i);
    scans{i} = lidarScan(double([xyz(:,1),xyz(:,3)]));
    pause(1);
end

%% Release the devices
release(colorDevice);
release(depthDevice);
