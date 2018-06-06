translation = []
i=3;
while 1
   while exist(strcat('D:\VirtualBox\share\textdata\',num2str(i),'.txt'))==0
       i = i+1;
   end
   
      fname = strcat('D:\VirtualBox\share\textdata\',num2str(i-1),'.txt');
   
   if exist(fname)>1
       fileID = fopen(fname,'r');
       translation = [translation fscanf(fileID,'[%f,%f,%f]')];
       fclose(fileID);
       %delete(fname);
   end
   i = i+1;
   [m,n] = size(translation);
   if n>1150
       break
   end
end

plot3(translation(2,1:1150),translation(1,1:1150),translation(3,1:1150))
axis equal
