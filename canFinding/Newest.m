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

% colorImage = imread('Cola130.jpg');
% depthImage = imread('Depth130.jpg');

I  = step(colorDevice);
%     I = imread('Cola180.jpg');
% Filter in only red stuff
I_hsv = rgb2hsv(I);

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

maxNumObj = 2;
hBlob = vision.BlobAnalysis('MaximumCount', maxNumObj, 'ExcludeBorderBlobs', true);

% Set the minimum size for blobs
hBlob.MinimumBlobArea = 500;
% Set the max size. You should do this a bit more robustly and smartly.
hBlob.MaximumBlobArea = 100000;

% The step command is apparently the way to apply the blob tracking on the
% image. The bounding box numbers are: [minX, minY, width, height]
%     [area centroid bbox]  = step(hBlob, I_can_closed);
%     [row,col]=size(bbox);

%%
% find both black and white regions
%     stats = [regionprops(I_can_closed,'orientation', 'BoundingBox','Centroid', 'extrema', 'area')];


stats = [regionprops(cansBinaryImage,'Centroid', 'area', 'BoundingBox', 'PixelIdxList')];
figure(1);
hold on;
imshow(not(cansBinaryImage));
figure(2);
hold on;
imshow(colorImage);

% Increment for creating an array of can positions
iii = 0;
for ii = 1:length(stats)
    if stats(ii).Area > 100
        % Increment counter
        iii = iii + 1;
        
        % Get a fresh clear image
        colorImage1=uint8(ones(1080,1920,3).*255);
        
        % Plot red outlines of cans
        figure(1);
        rectangle('Position', stats(ii).BoundingBox,'Linewidth', 3, 'EdgeColor', 'r', 'LineStyle', '--');
        figure(2);
        rectangle('Position', stats(ii).BoundingBox,'Linewidth', 3, 'EdgeColor', 'r', 'LineStyle', '--');
        
        % Change colorImage1 pixels that are part of the can from white to
        % black(255 to 0)
        temp = colorImage1(:,:,1);temp(stats(ii).PixelIdxList) = 0;colorImage1(:,:,1) = temp;
        temp = colorImage1(:,:,2);temp(stats(ii).PixelIdxList) = 0;colorImage1(:,:,2) = temp;
        temp = colorImage1(:,:,3);temp(stats(ii).PixelIdxList) = 0;colorImage1(:,:,3) = temp;
        
        % Feed the binary image into a point cloud function
        % to let the function
        % figure out the x, y, and z positions.
        ptCloud = pcfromkinect(depthDevice,depthImage,colorImage1);
        
        
        % Isolate the points that are part of the can(by finding colours
        % that = 0), then take the average of their x, y, and z positions
        k=find(ptCloud.Color(:,:,1)==0);
        temp=ptCloud.Location(:,:,1);temp=temp(k);temp(isnan(temp)) = []; objectPos(1,iii) = mean(temp);
        temp=ptCloud.Location(:,:,2);temp=temp(k);temp(isnan(temp)) = []; objectPos(2,iii) = mean(temp);
        temp=ptCloud.Location(:,:,3);temp=temp(k);temp(isnan(temp)) = []; objectPos(3,iii) = mean(temp);
        
        % Display text next to the cans to show us their positions
        figure(1);
        text(round(stats(ii).Centroid(1))+30,round(stats(ii).Centroid(2))+20, sprintf('x: %.2fm, y: %.2fm, z: %.2fm',objectPos(1,iii),objectPos(2,iii),objectPos(3,iii)));
        figure(2);
        text(round(stats(ii).Centroid(1))+30,round(stats(ii).Centroid(2))+20, sprintf('x: %.2fm, y: %.2fm, z: %.2fm',objectPos(1,iii),objectPos(2,iii),objectPos(3,iii)));
    end
end
figure(3);
imshow(depthImage.*10);

