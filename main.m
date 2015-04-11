% INDIVIDUAL POINT, MOTION PRIMITIVE SIMULATOR v1.2
% it mapped a rainbow path of quiver circles, but that feature was removed
% =(
% Arun Das, Gabriel Ongpauco, Mojdeh Shahidi, Steven Waslander
% Last change: Gabriel Ongpauco, 2012/07/7

% adjustable simulation parameters:
% map file
% duration
% physical integration timestep (should be smaller than program loop rate)
% snapshot interval (every x seconds)
% starting position
% initial heading


% adjustable robot parameters (WATHOG)
% velocity (vel)
% dt
% robotLoopRate
% collision check box geometry (create completely new geometry for new
% robots)


close all; clear all;clc; %close all figures, clear all variables, clear the screen
enableScoreReadout = false; % if you want to see the scores at each major point


%code parameters
mapfile = 'testMap2.bmp'; %location of the input map file

%map parameters
% global state [x y theta] of the robot on the map; centre of the robot?
testX = 1250; %input('Test point X coordinate: ');
testY = 1350; %input('Test point Y coordinate: ');
testHeading = 30; %input('Test point heading, deg. (0 = straight up, CCW +): ');

robotState = [ testX testY -1*(pi/2 + degtorad(testHeading)) ]; %global state [x y theta] of the robot on the map; centre of the robot?
lastSteeringAngle = 0; % leave at 0 unless you want to torture test the algorithm (odd cases)

% scoring params
weight_length = 1;
weight_pathlength = 1;
weight_PE = -125;
weight_mindistvar = -1/850;
weight_angle = 100;


%lidar parameters
robotWidth = 36; % in cm; GUESSTIMATE
robotLength = 58;
rmax = 1000; %max range of lidar (in cm);
rangeRejectMax = rmax-50; %max range to reject
rangeRejectMin = 10; %min range to reject
lidarResolution =  0.0044; % lidar angular resolution (in Radians)
lidarMinAngle = -2.3562; % minimum angle range of lidar (radians)
lidarMaxAngle = 2.3562; % maximum angle range of lidar (radians)

measPhi = lidarMinAngle:lidarResolution:lidarMaxAngle; %generate an array of lidar measurement headings

%vehicle parameters
lidarToAxle_dx = 0;% x component of lidar origin to the centre of the front axle (centre point of front axle is the origin for motion primitives) (i think)
lidarToAxle_dy = 15;% y component (cm)
lidarToAxle_dth = 0; % rotation between the two frames (rad)
WHEELBASE = 30; %cm
dt = 1/20 ; %integration time step, 20Hz
robotLoopRate = 1/20;
tmax = 5; %max integration time;
vel = 400; %vehicle speed cm/s, constant for now
steerMin = degtorad(-25); %min steering angle; IN RADIANS
steerMax = degtorad(25); %max steering angle; IN RADIANS
steerResolution = degtorad(1); %resolution of steering for motion primative generation

% calculation parameters
boxSafetyFactor = 2;

% clustering parameters
rangeDiffThreshold = 20; % cm I think

% scoring params
weight_length = 1;
weight_pathlength = 1;
weight_PE = -125;
weight_mindistvar = -1/850;
weight_angle = 100;

% simulation parameters
timeElapsed = 0;
simulationDuration = 7; % seconds
snapshotInterval = 1; % every X seconds, robot footprint and heading is plotted
simulationTimestep = 1/200; % for integration along simulated path



simulatedPath = []; % allocate array for simulated path
% map box corners, mapped to "MATLAB clustered frame" (forwards is pi/2)
% [x y];
robotBox = [-1*robotWidth/2 robotLength/2]; % front left
robotBox = [robotBox; robotWidth/2 robotLength/2]; % front right
robotBox = [robotBox; robotWidth/2 -1*robotLength/2]; % back right
robotBox = [robotBox; -1*robotWidth/2  -1*robotLength/2]; % back left
axleToCentre = [0, -WHEELBASE/2]; % add this to robotBoxSafety later on to properly get the centre

% scale collision box up using factor of safety
robotBoxSafety = boxSafetyFactor * robotBox;

% now put the box corners to the axle's coordinate frame
collisionBoxAxleFrame = [];

for i=1:length(robotBoxSafety)
    
    % add the offset vector to each corner vector
    % rotating these corners will now rotate them about the axle's midpoint
    collisionBoxAxleFrame = [collisionBoxAxleFrame; robotBoxSafety(i,:) + axleToCentre];

end


%load the map from file
trackMap = readMap(mapfile);

%plot the map
scrsz = get(0,'ScreenSize');
figure('OuterPosition',[scrsz(3)*3/5 scrsz(4)/10 scrsz(3)*2/5 scrsz(4)*4/5])
subplot(2,2,1)
whitebg('black');
spy(trackMap);
title('track map (global co-ordinate frame)');
grid on;
hold on; % tight! ...lame joke.
set(gca,'XMinorTick', 'on', 'YMinorTick', 'on');
xlabel('x')
ylabel('y')
axis square;
% robot plotting!



% start the simulation timer
simulationClock = 0;


while (simulationClock <= simulationDuration) % time limit on simulation

    snapshotTime = (mod( simulationClock, snapshotInterval ) < robotLoopRate); % is it time to update the graphs?
    
if ( snapshotTime )
    subplot(2,2,1)
    
    
    scatter(robotState(end,1),robotState(end,2),100,'LineWidth',2);
    quiver(robotState(end,1),robotState(end,2),cos(robotState(end,3)),sin(robotState(end,3)),100, 'LineWidth', 2); %is this the arrow thing that appears on the LIDAR's centre?
end

% CLUSTERING CODE
% FIXME GTO
% encapsulate this for later, it's ugly here!
%generate a lidar scan

lidarRanges = getranges(trackMap',robotState(end,1:3),measPhi,rmax);
numLidarRanges = length(lidarRanges); %number of lidar scan points


%convert to xy
lidarRawXY = []; % allocate array for x,y
lidarRawCluster = [];
find_centroid = 0;
sum_ranges = 0;
sum_thetas = 0;
avg_theta = 0;
avg_range = 0;
num_pts = 0;

for i=2:numLidarRanges

    tempRangeDiff = abs( lidarRanges(i) - lidarRanges(i-1));
    
     if (lidarRanges(i) <= rangeRejectMax && lidarRanges(i) >= rangeRejectMin) %make sure the range is within the min and max rejection range
        lidarRawXY = [lidarRawXY; lidarRanges(i)*sin(measPhi(i)) lidarRanges(i)*cos(measPhi(i)) ]; % fill LIDAR scan graph first
     
     end
     
     if tempRangeDiff > rangeDiffThreshold % if (dist >= delta) -> found the beginning of a peak
        if find_centroid == 1
            avg_theta = sum_thetas/num_pts;
            avg_range = sum_ranges/num_pts;
            lidarRawCluster = [lidarRawCluster; avg_range*sin(avg_theta) avg_range*cos(avg_theta) ];
            % reset sums of angles and ranges to zero
            sum_ranges = 0;
            sum_thetas = 0;
        end
        
    
        % ramp detection to come later
    
        if (lidarRanges(i) <= rangeRejectMax && lidarRanges(i) >= rangeRejectMin) %make sure the range is within the min and max rejection range
            % initialize centroid finding
            lidarRawXY = [lidarRawXY; lidarRanges(i)*sin(measPhi(i)) lidarRanges(i)*cos(measPhi(i)) ]; % fill LIDAR scan graph first
            sum_thetas = measPhi(i);
            sum_ranges = lidarRanges(i);
            find_centroid = 1;
            num_pts = 1;
               
        else % out of range limit
            find_centroid = 0; % ignore this peak
        end
    
    else
        if find_centroid == 1
            sum_thetas = sum_thetas + measPhi(i);
            sum_ranges = sum_ranges + lidarRanges(i);
            num_pts = num_pts + 1;
        end
    end
    
end

% DEBUG
% display the raw XY coordinates if needing to debug
% display(lidarRawXY)


% plot the laser scan (laser co-ordinate frame)
% figure(2)
%if ( snapshotTime )
subplot(2,2,2)
hold off
scatter(lidarRawXY(:,1), lidarRawXY(:,2),'r');
grid on;
title('lidar scan (lidar co-ordinate frame)');
axis square;
xlabel('x')
ylabel('y')
%end

%convert the laser scan to the axle-co-ordinate frame
numClusters = length(lidarRawCluster);
R = [cos(lidarToAxle_dth) -sin(lidarToAxle_dth); sin(lidarToAxle_dth) cos(lidarToAxle_dth)];
L = R*[lidarRawCluster(:,1)' ; lidarRawCluster(:,2)' ] + repmat( [lidarToAxle_dx;lidarToAxle_dy] , 1,numClusters);
lidarAxleCluster = [ L(1,:)' L(2,:)' ];


% for now, consider a cluster as a pylon
numPylons = numClusters;

%plot clustered axle frame
% figure(4)
%if ( snapshotTime )
subplot(2,2,3); hold off;
% find a way to plot the actual point and have it show up with the pylons!

scatter(lidarAxleCluster(:,1),lidarAxleCluster(:,2), 'CData',[ 1 0.5 0]); % orange after a dozen tries to get the parameter syntax right

title('pylon clusters (axle frame)');
grid on;
hold off;
xlabel('x')
ylabel('y')
axis square;

% plot motion primitives
subplot(2,2,4); hold off;
% find a way to plot the actual point and have it show up with the pylons!

scatter(lidarAxleCluster(:,1),lidarAxleCluster(:,2), 'CData',[ 0.5 0.25 0]); % orange after a dozen tries to get the parameter syntax right

title('clusters, motion primitives (axle frame)');
grid on;
hold on;
axis square;
xlabel('x')
ylabel('y')

%end

steerVector  = steerMin:steerResolution:steerMax; %vector of steering angles for motion primatives
numMotionPrimitives = length(steerVector); %total number of primatives
% loop for paths, counter is i

% DEBUG scorematrix for display purposes only
if enableScoreReadout
    scoreMatrix = []; % for individually displaying
end
motionPrimitiveScores = [];
for i=1:numMotionPrimitives
 
    motionPrimitive = []; % of form [ x y th ], th being the heading
    hasntHitSomething = 1; % LOL
    pointClearCount = 0;
    
    score_PE = 0;
    score_pointlength = 0;
    score_length = 0;
    score_mindistvar = 0;
    score_angle = 0;
    potentialEnergyTotal = 0; % defined here for scope
    
    % and for later, when we can simulate IMU + GPS code
    % score_GPS = 0;
    
    minDistArray = []; % list of all min dist's to obstacles for each point; find the maximum and "variance" of this array
    
    lastPathPoint = [ 0, 0, 0 ]; % motion primitives start at origin (axle midpoint)
    motionPrimitive = lastPathPoint;
    pathLength = 0;
    % loop for points
    while hasntHitSomething == 1
        % add new state row to bottom
       
        
        % map the collision box
        % corner vectors in rows; absolute coordinates, axle frame (or
        % whatever frame you like, really)
        
        % below is commented out due to issues with collision box mapping
        collisionBox = constructCollisionBox(lastPathPoint(1,1:2), lastPathPoint(1,3), collisionBoxAxleFrame);
   
        hasntHitSomething = collisionCheck(collisionBox, lidarAxleCluster); % again, LOL
        % circular body collision detection (using closest distance only)
        %hasntHitSomething = ~(circularCollisionDetection(lastPathPoint(1:2), lidarAxleCluster));
        
        % the following second condition is a temporary infinite loop
        % prevention
        limit_pointlength = 30;
        if ((hasntHitSomething == 1) && abs(motionPrimitive(end,3)) < degtorad(180) && (pointClearCount <= limit_pointlength)) % only do following calculations if collision check has been passed
            motionPrimitive = [ motionPrimitive; genMotionPrimitivePoint(lastPathPoint,steerVector(i),dt,vel,WHEELBASE,robotLength) ]; % using i as indice for steerVector (i counts primitives as well)        
            
            % score data calculations
            pointClearCount = pointClearCount + 1;
            pathLength = pathLength + calculateDistance(motionPrimitive(end,1:3) - lastPathPoint);
            minDistArray = [ minDistArray; minimumDistance(lidarAxleCluster,motionPrimitive(end,1:2)) ]; % use the last entry in the motion primitive matrix to calculate next min dist.
            potentialEnergyTotal = potentialEnergyTotal + calculatePotentialEnergy(motionPrimitive(end,1:2), lidarAxleCluster); % only use columns 1 & 2 (x & y) of motionPrimitive for vector stuff 
            
            lastPathPoint = motionPrimitive(end,1:3); % pass the last point to the motion primitive point generator
            % get ready to pass the last generated point to the motion
            % primitive generator again
            
        else
            hasntHitSomething = 0;
        end
        
    end % path has collided with an obstacle
    
    % steerVector already has all the headings
    
    % scoring!
    score_pointlength = pointClearCount;
    score_length = pathLength;
    score_mindistvar = calculateMinDistVariance(minDistArray);
    score_PE = potentialEnergyTotal;
    score_angle = -1*abs(steerVector(i)-lastSteeringAngle); % negative for subtraction; using 1/score may have 1/0 resulting in infinity
    
   
    % final scoring for the path
    % linear index i is keeping track of both the paths and scores in their
    % respective arrays
    pathScore = weight_length*score_length + weight_mindistvar*score_mindistvar + weight_PE*score_PE + weight_angle*score_angle ; % weight_GPS*score_GPS ; % don't forget the GPS score!
    
    % individual scoring display matrix
    % fill left-to-right
    % DEBUG scorematrix for debug/verbose output only
    if enableScoreReadout
        scoreMatrix = [radtodeg(steerVector(i)) score_pointlength weight_pathlength*score_length weight_mindistvar*score_mindistvar weight_PE*score_PE weight_angle*score_angle, pathScore; scoreMatrix ];
    end
    
    motionPrimitiveScores = [ motionPrimitiveScores pathScore ]; % add scores to the end of the row
    motionPrimitiveStores{i} = motionPrimitive;
    
    %if ( snapshotTime )
    subplot(2,2,4)
    plot(motionPrimitive(:,1),motionPrimitive(:,2),'Color',[ 0 0.5 1]);
    axis square;
    %end

end
% disp(steerVector) % 2012/06/30: steerVector seems to be properly populated

if enableScoreReadout
    
    if (snapshotTime )
        fprintf('Path scores at t = %f seconds \n ', simulationClock)
        disp('  Heading (d)   Half-steps  Length (cm)   MinDistVar         PE   Steering (r)   Total Score')
        format shortG
        disp(scoreMatrix)

        axis square;
    end 
end

% Set new robot state
[bestScore,bestMotion] = max(motionPrimitiveScores); % bestMotion takes the index of the bestScore!
curMP = motionPrimitiveStores{bestMotion};

% record the last steering angle that was sent
lastSteeringAngle = steerVector(bestMotion);
%step = min(length(curMP(:,1)),horizonStep);
%rotatedVec = rotateVector([curMP(step,2) curMP(step,1)],-robotState(3));
%robotState = robotState + [rotatedVec(1) rotatedVec(2)  -curMP(step,3)]; %global state [x y theta] of the robot on the map; centre of the robot?

% motion primitive equation: bicycle steering
% hold speed and steering constant, but integrate over a different (finer) dt for the
% actual robot path simulation
simulationTimestepCount = 0; % keep a running count until you hit the robotLoopRate
graphCurveUpdate = [];
while (simulationTimestepCount <= robotLoopRate)
    % don't even ask why this is so long. It's because of the CSYS of the
    % map file.
    
    deltaX = (-1)*vel*sin((-1)*robotState(end,3) - pi/2);
    deltaY = (-1)*vel*cos((-1)*robotState(end,3) - pi/2);
    deltaTH = (-1)*(vel/WHEELBASE)*tan(steerVector(bestMotion));
    
    %use the same formulas for both the graph update and actual simulated
    %path; only map the graph curve update every time
    graphCurveUpdate = [graphCurveUpdate; robotState(end,1) + (deltaX)*simulationTimestep, robotState(end,2) + (deltaY)*simulationTimestep, (robotState(end,3) + (deltaTH)*simulationTimestep)];
    robotState = [ robotState; robotState(end,1) + (deltaX)*simulationTimestep, robotState(end,2) + (deltaY)*simulationTimestep, (robotState(end,3) + (deltaTH)*simulationTimestep)];
    simulationTimestepCount = simulationTimestepCount + simulationTimestep;
end

subplot(2,2,1)
plot( graphCurveUpdate(:,1), graphCurveUpdate(:,2), 'Color', [1 1 1] );

%if ( snapshotTime )
subplot(2,2,4)
% plot on the motion primitives diagram the chosen path
plot(curMP(:,1),curMP(:,2),'Color',[ 1 1 1]);



% draw the robot's path on the global coordinate frame


%subplot(2,2,1)
%plot( robotState(:,1), robotState(:,2), 'Color', [1 1 1] );

drawnow; % Update plot now
%end
% pass enough time for the ROS loop to continue (1/20 s)
simulationClock = simulationClock + robotLoopRate;



end % repeat
