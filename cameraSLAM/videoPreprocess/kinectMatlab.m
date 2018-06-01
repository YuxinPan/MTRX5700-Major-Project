%release(colorDevice);
%release(depthDevice);

colorDevice = imaq.VideoDevice('kinect',1)

depthDevice = imaq.VideoDevice('kinect',2)


step(colorDevice);
step(depthDevice);

colorImage = step(colorDevice);  
depthImage = step(depthDevice);

% ptCloud = pcfromkinect(depthDevice,depthImage,colorImage);
% 
% player = pcplayer(ptCloud.XLimits,ptCloud.YLimits,ptCloud.ZLimits,...
% 	'VerticalAxis','y','VerticalAxisDir','down');
% 
% xlabel(player.Axes,'X (m)');
% ylabel(player.Axes,'Y (m)');
% zlabel(player.Axes,'Z (m)');
% 

for i = 1:inf    
   colorImage = step(colorDevice);  
   depthImage = step(depthDevice);
   colorImage = flipdim(colorImage ,2);
   depthImage = flipdim(depthImage ,2);
   %resizedColorImage = imresize(colorImage,0.25);
   resizedColorImage2 = imcrop(colorImage,[308 0 1304 1080]);

   %imwrite(resizedColorImage2,strcat(strcat('temp/',num2str(i),'.jpg')))
   %imwrite(colorImage,strcat(strcat('temp/',num2str(i),'O.jpg')))

   resizedColorImage = imresize(resizedColorImage2, [424 512]);

   imwrite(resizedColorImage,strcat(strcat('D:\VirtualBox\share\rgb\',num2str(i),'.jpg')))
   imwrite(depthImage,strcat(strcat('D:\VirtualBox\share\depth\',num2str(i),'.png')))
   
% 
%    ptCloud = pcfromkinect(depthDevice,depthImage);
%    x = reshape(ptCloud.Location(:,:,1),[],1)';
%    y = reshape(ptCloud.Location(:,:,2),[],1)';
%    z = reshape(ptCloud.Location(:,:,3),[],1)';
%    xyz = [x; y; z];
%    k = find(ptCloud.Location(:,:,2) > -0.30); 
%    xyz = xyz(:,k);
%    k = find(xyz(2,:) < -0.10); 
%    xyz = xyz(:,k);
%    
%    view(player,ptCloud);
% %    figure(1);
%    hold off;
%    grid on;
%    plot(xyz(1,:), xyz(3,:),'.');
%    xlabel('x');
%    ylabel('y');
% %    zlabel('z');
%     axis equal;
%     axis([-5 5 0 9]);
%     
%     pause(1);
%    figure(2);
%    subplot(1,2,1);
%    imshow(depthImage);
%    subplot(1,2,2);
%    imshow(colorImage);
end
%%
release(colorDevice);
release(depthDevice);
