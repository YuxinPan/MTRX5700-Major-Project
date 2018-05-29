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
    grid on;
for i = 1:500
    colorImage = step(colorDevice);
    depthImage = step(depthDevice);
    
    ptCloud = pcfromkinect(depthDevice,depthImage);
    x = reshape(ptCloud.Location(:,:,1),[],1)';
    y = reshape(ptCloud.Location(:,:,2),[],1)';
    z = reshape(ptCloud.Location(:,:,3),[],1)';
    xyz = [x; y; z];
    k = find(ptCloud.Location(:,:,2) > -0.30);
    xyz = xyz(:,k);
    k = find(xyz(2,:) < -0.28);
    xyz = xyz(:,k);
    
%     view(player,ptCloud);

    plot(xyz(1,:), xyz(3,:),'.');
    xlabel('x');
    ylabel('y');
    %    zlabel('z');
    axis equal;
    axis([-5 5 0 9]);
    drawnow;
    

end
%% Release the devices
release(colorDevice);
release(depthDevice);