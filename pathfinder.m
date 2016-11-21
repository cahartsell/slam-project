function [ desired_heading ] = pathfinder( current_map, robot_pos, target, SAVE_FILE )
%PATHFINDER Summary of this function goes here
%   Detailed explanation goes here
    
    % NOTE: Can't load variables from workspace into a function which uses
    %       nested functions. Need to fix this
    %pathfinder Macros
    MAP_SIZE            = 11; %Should be odd number
    TILE_SIZE           = 5;
    
    % Initialize map
    map = zeros(MAP_SIZE, MAP_SIZE);

    [ num_walls, ~ ] = size( current_map );
    for i = 1:num_walls
       points = find_wall_points( current_map(i,:) );
    end
    
    % NOTE: imagesc() function may help visualize map

    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                       Helper Functions                        %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % find_wall_points takes start and end coordinates of a wall and
    % calculates several points along the wall. Step size is <= TILE_SIZE,
    % which garuntees at least one point in every map tile the wall crosses
    function [ points ] = find_wall_points( wall ) 
       dx = wall(3) - wall(1);
       dy = wall(4) - wall(2);
       wall_len = sqrt( dx^2 + dy^2 );
       num_points = ceil( wall_len / TILE_SIZE );
       x_step = dx / (num_points - 1);
       y_step = dy / (num_points - 1);
       points = zeros(2, num_points);
       
       wallXSteps = x_step * (0:(num_points-1));
       wallYSteps = y_step * (0:(num_points-1));
       points(1, :) = repmat(wall(1), 1, num_points) + wallXSteps;
       points(2, :) = repmat(wall(2), 1, num_points) + wallYSteps;
    end
end

