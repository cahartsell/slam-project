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
DIST_WEIGHT         = 1000;
VISUALIZE_MAP       = 1;
VISUALIZE_PATH      = 1;
VIS_MAP_ALPHA       = 0.3; % Transparency percentage

% getLidar Macros
NUM_LIDAR_LINES     = 50;
LIDAR_RANGE         = 40;
LIDAR_STD_DEV       = 4;
LIDAR_BIAS          = 0;

% Breezy SLAM Macros
MAP_SIZE_PIXELS          = ENVIRONMENT_SIZE * 1;
MAP_SIZE_METERS          = ENVIRONMENT_SIZE / 10;
ROBOT_SIZE_PIXELS        = 10;

% Save macros to file so all functions can access. Overwrite old file
save( SAVE_FILE );
generateEnvironment( SAVE_FILE );
load( SAVE_FILE );

lidarRays = getLidar( robot_start(1), robot_start(2), wall_map );
temp = lidarRays < 40;
lidarRays = lidarRays .* temp;

% Sample path for robot
pos = zeros(240,2);
pos(1,1) = robot_start(1);
pos(1,2) = robot_start(2);
for i = 2:80
    pos(i,1) = pos(i-1,1)+1;
    pos(i,2) = pos(i-1,2);
end
for i = 81:160
    pos(i,1) = pos(i-1,1);
    pos(i,2) = pos(i-1,2)+1;
end
for i = 161:240
    pos(i,1) = pos(i-1,1)-1;
    pos(i,2) = pos(i-1,2)-1;
end

% Define Laser
laser.scan_size = NUM_LIDAR_LINES;
laser.scan_rate_hz = 10;
laser.detection_angle_degrees = 359;
laser.distance_no_detection_mm = LIDAR_RANGE * 100;
laser.detection_margin = 10;
laser.offset_mm = 1;

% Initialize SLAM
start_pos(1) = robot_start(1) * 100;
start_pos(2) = (ENVIRONMENT_SIZE - robot_start(2)) * 100;
start_pos(3) = 0;
slam = Deterministic_SLAM(laser, MAP_SIZE_PIXELS, MAP_SIZE_METERS, start_pos);
    
for i = 1:240
    roboX = pos(i,1);
    roboY = pos(i,2);
    lidarRays = getLidar( roboX, roboY, wall_map );
    
    %Convert Lidar Rays to SLAM format
    temp = lidarRays < LIDAR_RANGE;
    temp = temp * 100;
    temp = lidarRays .* temp;
    mid = round(NUM_LIDAR_LINES/2);
    slamLidarRays(1:mid,:) = flip( temp(1:mid,:) );
    slamLidarRays(mid+1:NUM_LIDAR_LINES,:) = flip( temp(mid+1:NUM_LIDAR_LINES,:) );
    
    % Pathfinding
    map = zeros(MAP_SIZE, MAP_SIZE);
    %[ heading, map ] = pathfinder( wall_map, robot_start, target_pos, SAVE_FILE, map );
    
    % SLAM update
    slam = slam.update(slamLidarRays(:,1), [100.0, 0.0, 0.1]);

    % Get new position and map
    [x_mm, y_mm, theta_degrees] = slam.getpos();
    slam_map = slam.getmap();
    
    % Display Environment
    clf
    subplot(2,1,1);
    xlim([0, ENVIRONMENT_SIZE]);
    ylim([0, ENVIRONMENT_SIZE]);
    hold on
    for j=1:NUM_WALLS
       plot([wall_map(j,1), wall_map(j,3)], [wall_map(j,2), wall_map(j,4)], 'linewidth', 2);
    end
    plot(roboX, roboY, 'o');
    plot(target_pos(1), target_pos(2), '*');
    
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
    colormap('gray')
    hold on
    
    pause(0.1)
end

