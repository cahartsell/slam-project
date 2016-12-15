% Clean workspace
clc
clf
clear

% Add Breezy SLAM matlab folder to matlab path and recompile
BSLAM_path = [pwd '\BreezySLAM-master\matlab'];
path(path,BSLAM_path);
mex ./BreezySLAM-master/matlab/mex_breezyslam.c ./BreezySLAM-master/c/coreslam.c ./BreezySLAM-master/c/coreslam_sisd.c ./BreezySLAM-master/c/random.c ./BreezySLAM-master/c/ziggurat.c
% mex -g ./BreezySLAM-master/matlab/mex_breezyslam.c ./BreezySLAM-master/c/coreslam.c ./BreezySLAM-master/c/coreslam_sisd.c ./BreezySLAM-master/c/random.c ./BreezySLAM-master/c/ziggurat.c

% Global Macros
ENVIRONMENT_SIZE    = 120;
SAVE_FILE           = 'environment.mat';

% generateEnvironment Macros
MAX_WALL_LEN        = 60;
MIN_WALL_LEN        = 10;
NUM_WALLS           = 8;
NUM_WALL_POINTS     = 2;
MIN_TARGET_SEP      = 80;
WALL_EDGE_PAD       = 5;

%pathfinder Macros
TILE_SIZE           = 2; % Recommend even number
MAP_SIZE            = ENVIRONMENT_SIZE / TILE_SIZE; % This should be an integer
VISUALIZE_MAP       = 1;
VISUALIZE_PATH      = 1;
VIS_MAP_ALPHA       = 0.3; % Transparency percentage

% getLidar Macros
NUM_LIDAR_LINES     = 50;
LIDAR_RANGE         = 40;
LIDAR_STD_DEV       = 1;
LIDAR_BIAS          = 0;

% Breezy SLAM Macros
MAP_SIZE_PIXELS          = ENVIRONMENT_SIZE * 1;
MAP_SIZE_METERS          = ENVIRONMENT_SIZE / 10;
ROBOT_SIZE_PIXELS        = 10;

% Save macros to file so all functions can access. Overwrite old file
% Generate random environment and initialize map
save( SAVE_FILE );
generateEnvironment( SAVE_FILE );
load( SAVE_FILE );
map = zeros(MAP_SIZE, MAP_SIZE);

%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
% DEBUG
% Sample path for robot. Should be replaced by motion model
% pos = zeros(240,2);
% pos(1,1) = robot_start(1);
% pos(1,2) = robot_start(2);
% for i = 2:80
%     pos(i,1) = pos(i-1,1)+1;
%     pos(i,2) = pos(i-1,2);
% end
% for i = 81:160
%     pos(i,1) = pos(i-1,1);
%     pos(i,2) = pos(i-1,2)+1;
% end
% for i = 161:240
%     pos(i,1) = pos(i-1,1)-1;
%     pos(i,2) = pos(i-1,2)-1;
% end
%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%

% Initialize motion controller
priorValues = [0;0;0;0;0];
roboX = robot_start(1);
roboY = robot_start(2);
last_pos = robot_start;
steering_angle = robot_start(3);

% Define Laser
laser.scan_size = NUM_LIDAR_LINES;
laser.scan_rate_hz = 10;
laser.detection_angle_degrees = 360;
laser.distance_no_detection_mm = LIDAR_RANGE * 100;
laser.detection_margin = 10;
laser.offset_mm = 1;

% Initialize SLAM
start_pos(1) = robot_start(1) * 100;
start_pos(2) = (ENVIRONMENT_SIZE - robot_start(2)) * 100;
start_pos(3) = 0;
slam = Deterministic_SLAM(laser, MAP_SIZE_PIXELS, MAP_SIZE_METERS, start_pos);

for i = 1:240
    % Get current position and sample lidar
%     roboX = pos(i,1);
%     roboY = pos(i,2);
    if (roboX > ENVIRONMENT_SIZE) || (roboY > ENVIRONMENT_SIZE)
        break;
    end
    lidarRays = getLidar( roboX, roboY, wall_map );
    
    % Configure plot
    clf;
    subplot(2,1,1);
    colormap default;
    
    % Display Environment
    xlim([0, ENVIRONMENT_SIZE]);
    ylim([0, ENVIRONMENT_SIZE]);
    hold on
    for j=1:NUM_WALLS
       plot([wall_map(j,1), wall_map(j,3)], [wall_map(j,2), wall_map(j,4)], 'linewidth', 2);
    end
    plot(roboX, roboY, 'o');
    plot(target_pos(1), target_pos(2), '*');
    
    % Convert Lidar Rays to Breezy SLAM compatable format
    % Breezy SLAM takes data from -180 to 180 degrees
    % getLidar returns data from 0 to 360 degrees
    temp = lidarRays < LIDAR_RANGE;
    temp = temp * 100;
    temp = lidarRays .* temp;
    mid = round(NUM_LIDAR_LINES/2);
    slamLidarRays(1:mid,:) = flip( temp(1:mid,:) );
    slamLidarRays(mid+1:NUM_LIDAR_LINES,:) = flip( temp(mid+1:NUM_LIDAR_LINES,:) );
    
    % SLAM update position and map
    % velocities = [linear_speed_mm/s, angular_speed_deg/s, time_delta_s]
    velocities = findVelocities( [roboX, roboY, steering_angle], last_pos, 1);
    slam = slam.update(slamLidarRays(:,1), velocities);
    [x_mm, y_mm, theta_degrees] = slam.getpos();
    slam_map = slam.getmap();
        
    % Pathfinding
    [ heading, map ] = pathfinder( [roboX, roboY], target_pos, map, slam_map );
    
    % Display Lidar rays
    [numRays,~] = size(lidarRays);
    for j = 1:numRays
        X = roboX + lidarRays(j,1)*cos(lidarRays(j,2));
        Y = roboY + lidarRays(j,1)*sin(lidarRays(j,2));
        line([roboX;X],[roboY;Y],'Color',[1 0 0])
    end

    % Display map
    subplot(2,1,2);
    hold off
    image(slam_map/4) % Keep bytes in [0,64] for colormap
    hold on
    
    % Update steering controller
    last_pos = [roboX, roboY, steering_angle];
    [steering_angle, priorValues] = steer(heading,priorValues);
    
    % Update motion model
    [roboX,roboY,~,~] = motionModel(steering_angle, roboX, roboY);
    
    pause(0.3)
end

