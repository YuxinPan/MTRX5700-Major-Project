IMUserial = serial('COM3','BaudRate',115200)
fopen(IMUserial)


pause(2)

while 1
    fprintf(IMUserial,'r');
    text = fscanf(IMUserial)
    heading = str2num(extractAfter(text,"yaw:"))
end

fclose(IMUserial)
