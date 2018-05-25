colorDevice = imaq.VideoDevice('kinect',1)


depthDevice = imaq.VideoDevice('kinect',2)


step(colorDevice);
step(depthDevice);


colorImage = step(colorDevice);
depthImage = step(depthDevice);

ptCloud = pcfromkinect(depthDevice,depthImage,colorImage);

player = pcplayer(ptCloud.XLimits,ptCloud.YLimits,ptCloud.ZLimits,...
	'VerticalAxis','y','VerticalAxisDir','down');

xlabel(player.Axes,'X (m)');
ylabel(player.Axes,'Y (m)');
zlabel(player.Axes,'Z (m)');


for i = 1:500    
   colorImage = step(colorDevice);  
   depthImage = step(depthDevice);
 
   ptCloud = pcfromkinect(depthDevice,depthImage);
   x = reshape(ptCloud.Location(:,:,1),[],1)';
   y = reshape(ptCloud.Location(:,:,2),[],1)';
   z = reshape(ptCloud.Location(:,:,3),[],1)';
   xyz = [x; y; z];
   k = find(xyz(3,:) == 0); 
   view(player,ptCloud);
   
%    scatter3(x,y,z,'.');
%    pause(1);
%    figure(2);
%    subplot(1,2,1);
%    imshow(depthImage);
%    subplot(1,2,2);
%    imshow(colorImage);
end

release(colorDevice);
release(depthDevice);