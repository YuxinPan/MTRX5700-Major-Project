function f = Dijkstra(startPoint,endPoint,number_of_nodes,graph)

disp('Start Dijkstra search')

matrix(1,1:number_of_nodes)=Inf; % distance to reach is Inf
matrix(2,1:number_of_nodes)=-1; % the node before is currently unknown
matrix(3,1:number_of_nodes)=0; % 0 suggests this node is not solved

matrix(1,startPoint)=0; % distance to reach the starting point is 0
matrix(2,startPoint)=0; % equals 0 means no previous point
matrix(3,startPoint)=1; % start point solved

lastSolved = startPoint;

%signal = 0; % if some points are unreachable, as long as not end point, then it is fine

while ismember(0,matrix(3,:)) % if 0 exists in row 3, then not all solved

    for i=1:number_of_nodes
        % check if unsolved node, if so, if the newly calculated cost is
        % less than previous cost to the node
        if graph(lastSolved,i)<Inf && matrix(3,i)~=1 && (matrix(1,lastSolved)+graph(lastSolved,i))<matrix(1,i)
            matrix(1,i)=matrix(1,lastSolved)+graph(lastSolved,i);
            matrix(2,i)=lastSolved;
        end
    end
    
%     temp = []
%     while 1
%         
%         [~,index]=ismember(min(matrix(1,:)),matrix(1,:));
%         if matrix(3,index)==1 % if this node is already solved, then it is not the node to be looked at
%             temp = [temp index];  % this might not be correct
%         else
%             break
%         end
%     end

    matrix(4,:) = inf;
    for i=1:number_of_nodes  % find nodes that are not solved, mark in matrix(4,:)
        if matrix(3,i)==0  % not solved
            matrix(4,i)=matrix(1,i); % copy the distance to reach
        end
    end
    temp = inf;
    index=0;
    for i=1:number_of_nodes % find smallest distance, mark smallest node as solved
        if matrix(4,i)<temp
            index = i;
            temp = matrix(4,i);
        end
    end
    if temp==inf % if nothing found, then break as no route available
        break
    end
    

    % find min in the queue that is not solved yet, to mark as solved
%     copy = matrix; 
%     lastCopy = copy;
%     while 1
%         [~,index]=ismember(min(copy(1,:)),copy(1,:));
%         if copy(3,index)==1 % if this node is already solved, then it is not the node to be looked at
%             copy(1,index)=Inf;  % this might not be correct
%         else
%             break
%         end
%         %disp('here')
%         if lastCopy == copy % if stuck, then break, but don't return here, the end point might already be solved
%             signal =1;
%             break
%         end
%         lastCopy = copy;
%         
%         %matrix
%         
%     end
%     
%     if signal ==1  % if signals stuck, then break
%         break
%     end
    
    lastSolved = index;
    matrix(3,index)=1; % mark this min in the queue as solved
    if matrix(3,endPoint)==1 % if endPoint solved, no need for more calculation, optimization
        break
    end
    
    %disp(matrix)
end


    
    
if matrix(3,endPoint)==0 % if end point is unreachable, return
	disp('stuck')
	f = -1;
    return
end

fprintf('Min cost: %.2f\nPath: ',matrix(1,endPoint));
i=1;
path(i)=endPoint;
while 1 % find the shortest path by looking at reverse order
    i=i+1;
    if matrix(2,path(i-1))==0
        break
    else
        path(i)=matrix(2,path(i-1));
    end
end
path = fliplr(path);
disp(path)
f = path;
fprintf('End Dijkstra search\n\n')

end