

% Initialize communication
[serialObject] = RoombaInit(3)  % this is the serial port

while 1
travelDist(serialObject,0.5,1);
turnAngle(serialObject,0.1,-90);
travelDist(serialObject,0.5,1);

turnAngle(serialObject,0.1,90);
travelDist(serialObject,0.3,0.3);

turnAngle(serialObject,0.1,90);
travelDist(serialObject,0.3,0.3);

turnAngle(serialObject,0.1,90);
travelDist(serialObject,0.3,0.3);

turnAngle(serialObject,0.1,-90);
travelDist(serialObject,0.5,0.7);

turnAngle(serialObject,0.1,90);
travelDist(serialObject,0.5,1);

turnAngle(serialObject,0.1,-180);

end