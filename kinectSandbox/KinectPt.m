%% If we haven't released the devices, then release them
clear all;
close all;
clc;
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

ptCloud = pcfromkinect(depthDevice,depthImage);

gridStep = 0.2;
ptCloudOut = pcdownsample(ptCloud,'gridAverage',gridStep)

% player = pcplayer(ptCloud.XLimits,ptCloud.YLimits,ptCloud.ZLimits,...
%     'VerticalAxis','y','VerticalAxisDir','down');
% player2 = pcplayer(ptCloudOut.XLimits,ptCloudOut.YLimits,ptCloudOut.ZLimits,...
%     'VerticalAxis','y','VerticalAxisDir','down');

% xlabel(player.Axes,'X (m)');
% ylabel(player.Axes,'Y (m)');
% zlabel(player.Axes,'Z (m)');


    figure(1);
    hold off;
    grid on;
while 1
    colorImage = step(colorDevice);
    depthImage = step(depthDevice);
    
    ptCloud = pcfromkinect(depthDevice,depthImage);
    ptCloudOut = pcdownsample(ptCloud,'gridAverage',gridStep);
    
%     x = reshape(ptCloudOut.Location(:,:,1),[],1)';
%     y = reshape(ptCloudOut.Location(:,:,2),[],1)';
%     z = reshape(ptCloudOut.Location(:,:,3),[],1)';
    
    xyz = ptCloudOut.Location;
    k = find(xyz(:,2) > 0);
    xyz = xyz(k,:);
    k = find(xyz(:,2) < 0.2);
    xyz = xyz(k,:);
    fprintf('Point count: %d\n',size(xyz,1));
    

%     view(player,ptCloud);
%     view(player2,ptCloudOut);
    
    
    
    subplot(1,2,1);
    plot(xyz(:,1), xyz(:,3),'.');
    xlabel('x');
    ylabel('z');
    grid on;
    axis equal;
    axis([-5 5 0 9]);
    subplot(1,2,2);
    imagesc(flipdim(colorImage,2));
    axis equal;
    drawnow;
    

end
