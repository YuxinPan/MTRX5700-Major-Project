clc;
load('workspace4yuxin.mat')

myOccupancy = [];
myOccupancyCoordinate = [];
a = 0;
b = 0;

startX=3.3; % in plotting, x is horizontal
startY=-4.4;
endX=-3.8;
endY=2;

gridSize = 0.18;

% convert world frame to value close to 2D matrix index
startX = (startX-map.XWorldLimits(1))/gridSize;
startY = (startY-map.YWorldLimits(1))/gridSize;
endX = (endX-map.XWorldLimits(1))/gridSize;
endY = (endY-map.YWorldLimits(1))/gridSize;

for i=map.XWorldLimits(1):gridSize:map.XWorldLimits(2)
    a = a+1;
    for j=map.YWorldLimits(1):gridSize:map.YWorldLimits(2)
        b=b+1;
        myOccupancy(b,a) = getOccupancy(map,[i j]);
        if myOccupancy(b,a)<0.3
            myOccupancy(b,a)=0;
        else
            myOccupancy(b,a)=1;  % 1 is occupied
        end
        myOccupancyCoordinate(b,a,1) = i;
        myOccupancyCoordinate(b,a,2) = j;
    end
    b = 0;
end

myOccupancyShow = myOccupancy;
[m,n] = size(myOccupancy);
totalNodes = m*n
X = zeros(1,totalNodes);
Y = zeros(1,totalNodes);
V = zeros(1,totalNodes);

graph = ones(totalNodes);
graph = inf*graph .* (ones(totalNodes,totalNodes)-eye(totalNodes));
graph(isnan(graph))=0; % the diagnoal elements should be 0



temp = 0;
for i=1:m
    for j=1:n
        temp = temp+1;
        X(temp)=j;
        Y(temp)=i;
        V(temp)=myOccupancy(i,j);
        %if V(temp)==0
        %    temp
        %end
    end
end



knnMap = [X' Y'];
for i=1:totalNodes
    Point_Query = [X(i) Y(i)];
    if V(i)==1
        continue
    end
    Idx = knnsearch(knnMap,Point_Query,'k',8);
    [m1,n1] = size(Idx); % Idx is 1D array, but size function has to output 2 numbers
    for j=1:n1 % loop through n adjacent points of i
        if graph(i,Idx(j)) <inf % if already computed by point Idx(j) previously, skip, optimization
            continue
        end
        if V(Idx(j))==1
            continue
        end
        distance = sqrt((X(i) - X(Idx(j)))^2+(Y(i) - Y(Idx(j)))^2);
        graph(i,Idx(j))=1;
        graph(i,Idx(j)) = distance; 
        graph(Idx(j),i) = distance; % bidirectional map
    end
%     if i==8  # for debug use
%     	scatter(knnMap(Idx,1),knnMap(Idx,2),'r','filled') % plot neighbours
%     end
end


% find the node closest to start and end point
startNode = [startX startY];
startNodeIndex = knnsearch(knnMap,startNode,'k',1);
endNode = [endX endY];
endNodeIndex = knnsearch(knnMap,endNode,'k',1);

%%%% start finding the path  %%%%
path = Dijkstra(startNodeIndex,endNodeIndex,totalNodes,graph);
if path == -1
    fprintf('No Path');
else
    [m2,n2] = size(path);
    %params = [];
    for i=1:n2
        %params = [params;X(path(i)) Y(path(i))];
        row = floor(path(i)/n)+1;
        col = mod(path(i),n);
        myOccupancyShow(row,col)=0.5;
    end
    %[m3,n3] = size(params);
    %for i=1:(m3-1)
    %    plot([params(i,1) params(i+1,1)], [params(i,2) params(i+1,2)],'g','LineWidth',2)
    %end
end
%%%% end finding the path  %%%%


myOccupancyShow = flipud(myOccupancyShow);

figure();
imagesc(myOccupancyShow);
figure();
show(map);


% function nodeIndex = findNodeIndex(row,col,m,n)
%     nodeIndex = (row-1)*n+col;
% end