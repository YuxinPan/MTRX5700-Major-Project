clear
close all

dt = 1;

hold on
grid on
axis equal


% Sort it according to time


%%

% actual states
% x = [positionObs(1,3); positionObs(1,4)]

% estimator states
% Define our state as:
% [x;y;psi]
% initUncertainty = [0.1; 0.5];
x_est = [0;
    0;
    heading];
P = zeros(3,3);
% P(1,1) = initUncertainty(1,1)^2;
% P(2,2) = initUncertainty(2,1)^2;

% process model
% F = [1 dt; 0 1];
% v = [0.0; 0.01];
% Uncertainty in model state
Q = zeros(3,3);
Q(1,1) = 0.01^2;
Q(2,2) = 0.01^2;
Q(3,3) = 0.02^2;

% observation model
H = [1 0 0;
    0 1 0;
    0 0 1];

% Uncertainty in observations
R = [0.5^2 0 0;
    0 0.5^2 0;
    0 0 0.25^2];%m

% storage
x_store = x_est;
P_store = P;
inn_store = [];
S_store = [];

% Initialist dt trackers
lastVelTime = velocityObs(1,2);
lastPosTime = positionObs(1,2);
lastComTime = compassObs(1,2);

R = [0.5^2 0 0;
    0 0.5^2 0;
    0 0 0.25^2];%m


% Create a custom legend, because trying to play by the rules is too hard
h = zeros(6, 1);
h(1) = plot(NaN,NaN,'rx');
h(2) = plot(NaN,NaN,'bx');
h(3) = plot(NaN,NaN,'g.');
h(4) = plot(NaN,NaN,'b-');
h(5) = plot(NaN,NaN,'go');
h(6) = plot(NaN,NaN,'yo');
h(7) = plot(NaN,NaN,'y.');
%legend(h, 'GPS Positions','True Beacon Positions','Kalman Filter Position', 'Compass Angle Indicator', 'Confirmed Real Beacons', 'Observed Beacons', 'Beacon-derived Position', 'pos', 'best');


title('State Estimation');
xlabel('x / m');
ylabel('y / m');
%     M(ii) = getframe;
axis([-10 15 -10 10]);
for ii = 1:length(compiledData)
    % Input
    type = compiledData{ii}(1);
    
    % If we get velocity and rate of change of heading
    if type == 1
        
        time = compiledData{ii}(2);
        vk = compiledData{ii}(3);
        psikdot = compiledData{ii}(4);
        dt = time - lastVelTime;
        
        lastVelTime = time;
        
        
        
        F = [1 0 -dt * vk * sin(x_est(3));
            0 1 dt * vk * cos(x_est(3));
            0 0 1];
        P = F*P*F' + Q;
        
        
       
        % perform a prediction
        x_est = [x_est(1) + dt * vk * cos(x_est(3));
            x_est(2) + dt * vk * sin(x_est(3));
            x_est(3) + psikdot * dt];
        %         fprintf('Vel t: %.3d dt = %.3d vel: %.3d turn: %.3d state: %d\n',time,dt, vk, psikdot, x_est);
        if x_est(3) > pi
            x_est(3) = x_est(3) - 2 * pi;
        elseif x_est(3) < -pi
            x_est(3) = x_est(3) + 2 * pi;
        end
        
        p1 = plot(x_est(1), x_est(2), 'g.');
        
        % If we get GPS position observation
    elseif type == 2
        
        
        
        time = compiledData{ii}(2);
        xPos = compiledData{ii}(3);
        yPos = compiledData{ii}(4);
        %         fprintf('GPS t: %d xpos: %d ypox: %d\n',time, xPos, yPos);
        z = [xPos;
            yPos;
            x_est(3)];
        p2 = plot(z(1), z(2), 'rx');
        
        z_est = H*x_est;
        inn = z - z_est;
        inn_store = [inn_store inn];
        
        S = H*P*H' + R;
        S_store = [S_store S];
        
        W = P*H'*inv(S);
        
        x_est = x_est + W*inn;
        
        if x_est(3) > pi
            x_est(3) = x_est(3) - 2 * pi;
        elseif x_est(3) < -pi
            x_est(3) = x_est(3) + 2 * pi;
        end
        
        P = P - W*S*W';
        p1 = plot(x_est(1), x_est(2), 'g.');
%         
        %
        % If we get compass heading observation
    elseif type == 3
        
        time = compiledData{ii}(2);
        headingObs = compiledData{ii}(3);
        if headingObs > pi
            headingObs = headingObs - 2 * pi;
        elseif headingObs < -pi
            headingObs = headingObs + 2 * pi;
        end
        
        
        z = [x_est(1);
            x_est(2);
            headingObs];
        
        z_est = H*x_est;
        inn = z - z_est;
        
        if inn(3) > pi
            inn(3) = inn(3) - 2 * pi;
        elseif inn(3) < -pi
            inn(3) = inn(3) + 2 * pi;
        end
        
        inn_store = [inn_store inn];
        
        S = H*P*H' + R;
        S_store = [S_store S];
        
        W = P*H'*inv(S);
        
        x_est = x_est + W*inn;
        if x_est(3) > pi
            x_est(3) = x_est(3) - 2 * pi;
        elseif x_est(3) < -pi
            x_est(3) = x_est(3) + 2 * pi;
        end
        format bank
        fprintf('Comp t: %d headingObs: %2.1f headingEst: %2.1f\n',time, headingObs, x_est(3));
        P = P - W*S*W';
        p1 = plot(x_est(1), x_est(2), 'g.');
        
        
        m = headingObs;
        line([-anglePlotLength/2*cos(m) anglePlotLength/2*cos(m)]+x_est(1), [-anglePlotLength/2*sin(m) anglePlotLength/2*sin(m)]+x_est(2));
        
        % If it's laser data
    elseif type == 4
        time = compiledData{ii}(2);
        laserData = compiledData{ii}(3:length(compiledData{ii}));
        
        % Reformat into a more usable format, i.e. 2x361 array
        bufferLaserData = [];
        for iii = 1:length(laserData)/2
            bufferLaserData(:,iii) = [laserData(iii * 2 - 1);
                laserData(iii * 2)];
        end
        laserData = bufferLaserData;
        
        % Add in angle data to the array
        for iii = 1:length(laserData)
            angleDeg = -90 + iii/2;
            angleRad = deg2rad(angleDeg);
            
            laserData(3,iii) = angleRad; % Append angle data
        end
        
        % Remove readings of 8 metres
        for iii = length(laserData):-1:1
            if laserData(1,iii) == 8
                laserData(:,iii) = [];
            end
        end
        
        % Get approximate x-y positions at this point in time
        estBeaconPos = [];
        for iii = 1:length(laserData)
            absoluteAngle = laserData(3,iii) + x_est(3);
            if laserData(2,iii) == 1
                estBeaconPos = horzcat(estBeaconPos, [x_est(1) + laserData(1,iii) * cos(absoluteAngle);
                    x_est(2) + laserData(1,iii) * sin(absoluteAngle);
                    laserData(1,iii)]);
            end
        end
        
        % Plot the estimated beacons
        if length(estBeaconPos) > 0
            exist estBeaconPlot;
            if ans
                delete(estBeaconPlot);
            end
            estBeaconPlot = plot(estBeaconPos(1,:), estBeaconPos(2,:),'yo');
            
        end
        
        
        
        
        
        for iii = 1:size(estBeaconPos,2)
            
            % Find differences between observed beacons and all real beacons
            for iiii = 1:size(laserFeatures,1)
                dif(iiii) = sqrt((laserFeatures(iiii,1)-estBeaconPos(1,iii))^2 + (laserFeatures(iiii,2)-estBeaconPos(2,iii))^2);
            end
            % Put our estimated beacon number into the array too
            [finalDif estBeaconPos(4,iii)] = min(dif);
            if finalDif > beaconDetectThesh
                estBeaconPos(4,iii) = -1;
            end
            
        end
        
        % Find out if all estimated beacons were close enough to the real
        % beacons
        if length(estBeaconPos)
            if min(estBeaconPos(4,:)) == -1
                fprintf('No proper beacons found in set: %d\n',ii);
            else
                foundBeacons = laserFeatures(estBeaconPos(4,:),:);
                foundBeacons = unique(foundBeacons,'rows').';
                
                % Plot the positions of the real beacons found
                exist foundBeaconsPlot;
                if ans
                    delete(foundBeaconsPlot);
                end
                foundBeaconsPlot = plot(foundBeacons(1,:),foundBeacons(2,:),'go');
                
                beaconNumbersFound = unique(estBeaconPos(4,:)','rows');
                trueBeaconsPos = laserFeatures(beaconNumbersFound,:)';
                % Get an average observed position for each beacon
                aveObsBeacon = [];
                for iii = 1:length(beaconNumbersFound)
                    obsBeacons = find(estBeaconPos(4,:) == beaconNumbersFound(iii));
                    aveObsBeacon(:,iii) = [mean(estBeaconPos(1,obsBeacons));
                                           mean(estBeaconPos(2,obsBeacons));
                                           mean(estBeaconPos(3,obsBeacons))];
                end
                
                % Plot the estimated beacons
                
                exist estBeaconPlot;
                if ans
                    delete(estBeaconPlot);
                end
                estBeaconPlot = plot(aveObsBeacon(1,:), aveObsBeacon(2,:),'yo');
                

                % If we have found more than 2 beacons, get an x-y
                % position from the  beacons and update kalman filter
                if size(aveObsBeacon,2) > 3
                    
                    NLLS_return = NLLS_pos(aveObsBeacon(3,:), trueBeaconsPos, [x_est(1);x_est(2);0]);
                    fprintf('Beacon based regression x: %f y: %f\n',NLLS_return(1),NLLS_return(2));
                    plot(NLLS_return(1), NLLS_return(2), 'y.');
                    
                    
                    
                    
                    z = [NLLS_return(1);NLLS_return(2);x_est(3)];
                    z_est = H*x_est;
                    inn = z - z_est;
                    inn_store = [inn_store inn];
                    
                    S = H*P*H' + R;
                    S_store = [S_store S];
                    
                    W = P*H'*inv(S);
                    
                    x_est = x_est + W*inn;
     
                    x_est(3) = keepInBounds(x_est(3));
                    
                    P = P - W*S*W';
                    p1 = plot(x_est(1), x_est(2), 'g.');
                    

                end
            end
        end
        
        
    end
    
    
    
    pause(0.001)
    
    % store the current state estimates
    x_store = [x_store x_est];
    P_store = [P_store diag(P)];
    
    
end


function angleIn = keepInBounds(angleIn)
if angleIn > pi
    angleIn = angleIn - 2 * pi;
elseif angleIn < -pi
    angleIn = angleIn + 2 * pi;
end
end

% Find x-y position of robot using NLLS, probably not what was intended in
% the assignment, but it works.
function x0 = NLLS_pos(dists, beaconPos, initPos)

maxloops = 100;

x0 = initPos;
x = x0;


p0 = dists';
p = p0;

    

    
% Estimation
for ii = 1:maxloops
    
        % Set up matrix H
    for iii = 1:size(beaconPos,2)
        curBeacon = beaconPos(:,iii);
        R = sqrt((curBeacon(1)-x0(1))^2 + (curBeacon(2)-x0(2))^2);
        dpdx = -(curBeacon(1) - x0(1))/R;
        dpdy = -(curBeacon(2) - x0(2))/R;
        H(iii,:,ii) = [dpdx dpdy 1];
        
        p(iii, ii + 1) = R;
    end

   
        
   Dp0 = p0 - p(:,ii+1);
   
   Dx = inv(H(:,:,ii)'*H(:,:,ii))*H(:,:,ii)'*Dp0;
   
   x(:,ii+1) = x0 + Dx;
   
   x0 = x(:,ii+1);
   
   if abs(Dx(1)) < 1e-9 && abs(Dx(2)) < 1e-9
%        fprintf('Stopping at iteration: %d\n', ii);
       break;
   end
end
x0 = x0';

end