function [ Map ] = generateMap( )
%GENERATEMAP Summary of this function goes here
%   Detailed explanation goes here

    MAX_WALL_LEN = 60;
    MIN_WALL_LEN = 10;
    NUM_WALLS = 8;
    NUM_WALL_POINTS = 2;
    NUM_POINT_OBSTACLES = 0;
    
    pointObstacles = 140*rand(2,NUM_POINT_OBSTACLES)-70;
 
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Random generation of obstacle walls is vectorized for efficiency,
    % which makes it harder to follow logically
    wallLen     = (MAX_WALL_LEN - MIN_WALL_LEN) * rand(1, NUM_WALLS) + MIN_WALL_LEN;
    wallAngles  = 360 * rand(1, NUM_WALLS);
    wallPos     = 120*rand(2, NUM_WALLS)-60;
   
    wallXSteps = ((1/(NUM_WALL_POINTS-1)) * wallLen(1,:) .* cos(wallAngles(1,:)))' * (0:(NUM_WALL_POINTS-1));
    wallYSteps = ((1/(NUM_WALL_POINTS-1)) * wallLen(1,:) .* sin(wallAngles(1,:)))' * (0:(NUM_WALL_POINTS-1));
    
    wallXCoords = repmat(wallPos(1,:)', 1, NUM_WALL_POINTS) + wallXSteps;
    wallYCoords = repmat(wallPos(2,:)', 1, NUM_WALL_POINTS) + wallYSteps;
    
    wallXCoords = reshape( wallXCoords', [1 NUM_WALL_POINTS NUM_WALLS] );
    wallYCoords = reshape( wallYCoords', [1 NUM_WALL_POINTS NUM_WALLS] );
    
    Walls = zeros(2, NUM_WALL_POINTS, NUM_WALLS);
    Walls(1,:,:) = wallXCoords;
    Walls(2,:,:) = wallYCoords;
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    Map = Walls;
   
    plot( Map(1,:) , Map(2,:) , '.');
    hold on;
    for i=1:NUM_WALLS
       plot(Walls(1,:,i), Walls(2,:,i), 'linewidth', 2);
    end

end

