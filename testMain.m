% Clean workspace
clf
clc
clear

% Add Breezy SLAM matlab folder to matlab path and recompile
BSLAM_path = [pwd '\BreezySLAM-master\matlab'];
path(path,BSLAM_path);
mex ./BreezySLAM-master/matlab/mex_breezyslam.c ./BreezySLAM-master/c/coreslam.c ./BreezySLAM-master/c/coreslam_sisd.c ./BreezySLAM-master/c/random.c ./BreezySLAM-master/c/ziggurat.c
mex -g ./BreezySLAM-master/matlab/mex_breezyslam.c ./BreezySLAM-master/c/coreslam.c ./BreezySLAM-master/c/coreslam_sisd.c ./BreezySLAM-master/c/random.c ./BreezySLAM-master/c/ziggurat.c

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

% Save macros to file so all functions can access. Overwrite old file
save( SAVE_FILE );
generateEnvironment( SAVE_FILE );
load( SAVE_FILE );

lidarRays = getLidar( robot_start(1), robot_start(2), wall_map );
temp = lidarRays < 40;
lidarRays = lidarRays .* temp;
map = zeros(MAP_SIZE, MAP_SIZE);
%[ heading, map ] = pathfinder( wall_map, robot_start, target_pos, SAVE_FILE, map );

MAP_SIZE_PIXELS          = ENVIRONMENT_SIZE * 1;
MAP_SIZE_METERS          = ENVIRONMENT_SIZE / 1000;
ROBOT_SIZE_PIXELS        = 10;

laser.scan_size = NUM_LIDAR_LINES;
laser.scan_rate_hz = 10;
laser.detection_angle_degrees = 360;
laser.distance_no_detection_mm = LIDAR_RANGE;
laser.detection_margin = 70;
laser.offset_mm = 0;

slam = Deterministic_SLAM(laser, MAP_SIZE_PIXELS, MAP_SIZE_METERS);
% slam.setpos( robot_start(1), robot_start(2), robot_start(3));

slam = slam.update(lidarRays(:,1));

% Get new position
[x_mm, y_mm, theta_degrees] = slam.getpos();

% Get current map
map = slam.getmap();

% Display map
hold off
image(map/4) % Keep bytes in [0,64] for colormap
axis('square')
colormap('gray')
hold on

