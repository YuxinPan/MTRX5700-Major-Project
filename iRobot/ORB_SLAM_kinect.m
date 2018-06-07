release(colorDevice);
release(depthDevice);

colorDevice = imaq.VideoDevice('kinect',1)

depthDevice = imaq.VideoDevice('kinect',2)


step(colorDevice);
step(depthDevice);

colorImage = step(colorDevice);  
depthImage = step(depthDevice);

counter = 2;

% ptCloud = pcfromkinect(depthDevice,depthImage,colorImage);
% 
% player = pcplayer(ptCloud.XLimits,ptCloud.YLimits,ptCloud.ZLimits,...
% 	'VerticalAxis','y','VerticalAxisDir','down');
% 
% xlabel(player.Axes,'X (m)');
% ylabel(player.Axes,'Y (m)');
% zlabel(player.Axes,'Z (m)');
% 



% Initialize communication
[serialObject] = RoombaInit(18)  % this is the serial port

SetFwdVelRadiusRoomba(serialObject, 0.1, inf);

translation = [];

for i = 1:inf    
   colorImage = step(colorDevice);  
   depthImage = step(depthDevice);
   colorImage = flipdim(colorImage ,2);
   depthImage = flipdim(depthImage ,2);
   %resizedColorImage = imresize(colorImage,0.25);
   resizedColorImage2 = imcrop(colorImage,[308 0 1304 1080]);

   resizedColorImage = imresize(resizedColorImage2, [424 512]);

   imwrite(resizedColorImage,strcat(strcat('D:\VirtualBox\share\rgb\',num2str(i),'.jpg')))
   imwrite(depthImage,strcat(strcat('D:\VirtualBox\share\depth\',num2str(i),'.png')))
   
   while exist(strcat('D:\VirtualBox\share\textdata\',num2str(i),'.txt'))>1
       i = i+1
   end
   
   fname = strcat('D:\VirtualBox\share\textdata\',num2str(i-2),'.txt');
   
   if exist(fname)>1
       fileID = fopen(fname,'r');
       translation[] = fscanf(fileID,'[%f,%f,%f]')
       fclose(fileID);
       delete(fname);
   end



end
%%
release(colorDevice);
release(depthDevice);