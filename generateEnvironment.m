function [ wall_map, robot_start, target_pos] = generateEnvironment( )
%GENERATEMAP Summary of this function goes here
%   Detailed explanation goes here

    MAX_WALL_LEN        = 60;
    MIN_WALL_LEN        = 10;
    NUM_WALLS           = 8;
    NUM_WALL_POINTS     = 2;
    MAP_SIZE            = 120;
    MIN_TARGET_SEP      = 80;
    WALL_EDGE_PAD       = 5;
    
    % Initialize outputs
    wall_map = zeros( NUM_WALLS, (NUM_WALL_POINTS * 2) );
    robot_start = zeros(1, 3);
    target_pos = zeros(1, 2);
    
    robot_start(1:2) = MAP_SIZE * rand(1,2);
    robot_start(3) = 360 * rand();
    
    % Generate random target position until minimum seperation is met
    rt_seperation =0;
    while( rt_seperation < MIN_TARGET_SEP )
        target_pos(:) = MAP_SIZE * rand(1,2);
        dx = target_pos(1) - robot_start(1);
        dy = target_pos(2) - robot_start(2);
        rt_seperation = sqrt( dx^2 + dy^2 );
    end
    
 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Random generation of obstacle walls is vectorized for efficiency,
    % which makes it harder to follow logically.
    % 
    % Randomly generates sets of walls, until a set does not have any walls
    % which are outside the boundaries of the map. (Inefficient, but easy)
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    passed_check = 0;
    while( ~(passed_check) )
        wallLen     = (MAX_WALL_LEN - MIN_WALL_LEN) * rand(1, NUM_WALLS) + MIN_WALL_LEN;
        wallAngles  = 360 * rand(1, NUM_WALLS);
        wallOrigin  = ((MAP_SIZE - 2*WALL_EDGE_PAD) * rand(2, NUM_WALLS)) + WALL_EDGE_PAD;

        wallXSteps = ((1/(NUM_WALL_POINTS-1)) * wallLen(1,:) .* cos(wallAngles(1,:)))' * (0:(NUM_WALL_POINTS-1));
        wallYSteps = ((1/(NUM_WALL_POINTS-1)) * wallLen(1,:) .* sin(wallAngles(1,:)))' * (0:(NUM_WALL_POINTS-1));

        wallXCoords = repmat(wallOrigin(1,:)', 1, NUM_WALL_POINTS) + wallXSteps;
        wallYCoords = repmat(wallOrigin(2,:)', 1, NUM_WALL_POINTS) + wallYSteps;

        wallXCoords = reshape( wallXCoords', [1 NUM_WALL_POINTS NUM_WALLS] );
        wallYCoords = reshape( wallYCoords', [1 NUM_WALL_POINTS NUM_WALLS] );

        Walls = zeros(2, NUM_WALL_POINTS, NUM_WALLS);
        Walls(1,:,:) = wallXCoords;
        Walls(2,:,:) = wallYCoords;
        
        passed_check = 1;
        for i = 1:NUM_WALLS
           if( (Walls(1,2,i) < WALL_EDGE_PAD) || (Walls(1,2,i) > (MAP_SIZE - WALL_EDGE_PAD)) )
                passed_check = 0;
                break;
           end
           if( (Walls(2,2,i) < WALL_EDGE_PAD) || (Walls(2,2,i) > (MAP_SIZE - WALL_EDGE_PAD)) )
                passed_check = 0;
                break;
           end
        end
    end
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Walls(x,y,z) is addressed:    
    %   x = x-coord (1) or y-coord (2)
    %   y = Point along wall ( 1 to NUM_WALL_POINTS )
    %   x = Wall Number ( 1 to NUM_WALLS )
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % wall_map is NUM_WALLS x 4 array.
    % Each row contains start and end point for a single wall
    for i = 1:NUM_WALLS
        wall_map(i,1) = Walls(1,1,i);
        wall_map(i,2) = Walls(2,1,i);
        wall_map(i,3) = Walls(1,2,i);
        wall_map(i,4) = Walls(2,2,i);
    end
   
    % Plotting left in for easy visualization during development
    figure;
    xlim([0, MAP_SIZE]);
    ylim([0, MAP_SIZE]);
    hold on
    for i=1:NUM_WALLS
       plot([wall_map(i,1), wall_map(i,3)], [wall_map(i,2), wall_map(i,4)], 'linewidth', 2);
    end
    plot([robot_start(1), target_pos(1)], [robot_start(2), target_pos(2)], '*');
end

