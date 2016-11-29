function [ desired_heading ] = pathfinder( current_map, robot_pos, target, SAVE_FILE )
%PATHFINDER Summary of this function goes here
%   Detailed explanation goes here
    
    % NOTE: Can't load variables from workspace into a function which uses
    %       nested functions. Avoid nested functions
    load( SAVE_FILE );
    
    % Initialize map (centered on robot)
    map = zeros(MAP_SIZE, MAP_SIZE);
    map_origin(1) = robot_pos(1) - (TILE_SIZE * (MAP_SIZE/2));
    map_origin(2) = robot_pos(2) - (TILE_SIZE * (MAP_SIZE/2));
    [ num_walls, ~ ] = size( current_map );
    
    % Determine set of points along each wall from the current wall map. 
    dx = current_map(:,3) - current_map(:,1);
    dy = current_map(:,4) - current_map(:,2);
    wall_len = sqrt( dx.^2 + dy.^2 );
    num_points = ceil( wall_len / TILE_SIZE );
    max_num_points = max( num_points );
    wallXSteps = (dx ./ (max_num_points - 1)) * (0:(max_num_points-1));
    wallYSteps = (dy ./ (max_num_points - 1)) * (0:(max_num_points-1));
    points(1, :, :) = (repmat(current_map(:,1), 1, max_num_points) + wallXSteps)';
    points(2, :, :) = (repmat(current_map(:,2), 1, max_num_points) + wallYSteps)';
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % points(x,y,z) is addressed:    
    %   x = x-coord (1) or y-coord (2)
    %   y = Point along wall ( 1 to max_num_points )
    %   x = Wall Number ( 1 to num_walls )
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    
    % Convert wall points from global frame to map indicies
    % points_ind addressed the same as points
    points_ind(1,:,:) = points(1, :, :) - map_origin(1);
    points_ind(2,:,:) = points(2, :, :) - map_origin(2);
    points_ind = ceil( points_ind ./ TILE_SIZE );
    
    % Add gradient weighting to map which decreases with distance from target
    dx2 = repmat( (map_origin(1)+TILE_SIZE/2) , 1, MAP_SIZE) + (TILE_SIZE * (0:MAP_SIZE-1));
    dy2 = repmat( (map_origin(2)+TILE_SIZE/2) , 1, MAP_SIZE) + (TILE_SIZE * (0:MAP_SIZE-1));
    dx2 = dx2 - repmat( target(1), 1, MAP_SIZE );
    dy2 = dy2 - repmat( target(2), 1, MAP_SIZE );
    dx2 = dx2.^2;
    dy2 = dy2.^2;
    dist_from_target = repmat( dx2, MAP_SIZE, 1)' + repmat( dy2, MAP_SIZE, 1);
    dist_from_target = dist_from_target.^(1/2);
    denom_check = dist_from_target < 0.1;     % Prevent divide by zero
    dist_from_target = dist_from_target + denom_check;
    map = (dist_from_target.^(-1)) * DIST_WEIGHT;
    
    % Give negative weighting to map tiles with obstacles (walls)
    % Not vectorized. For loop is slow, but easy
    % Check that indicies are within range
    temp = points_ind > 0;
    points_ind = points_ind .* temp;
    temp = points_ind <= MAP_SIZE;
    points_ind = points_ind .* temp;
    for i = 1:num_walls
        last_x = points_ind(1,1,i);
        last_y = points_ind(2,1,i);
        for j = 1:max_num_points
            x_ind = points_ind(1,j,i);
            y_ind = points_ind(2,j,i);
            if (x_ind > 0) && (y_ind > 0)
                map( x_ind, y_ind ) = -1;
                % Simple (but not particularly precise) method to prevent
                % diagonal jumps through walls
                if (last_x > 0) && (last_y > 0) && (x_ind ~= last_x) && (y_ind ~= last_y)
                    map( last_x, y_ind ) = -1;
                    map( x_ind, last_y ) = -1;
                end
                last_x = x_ind;
                last_y = y_ind;
            end
        end
    end
    
    % Find best path using map
    % http://www.redblobgames.com/pathfinding/a-star/introduction.html
    found_path = 0;
    num_checked = 1;
    frontier(1,1) = MAP_SIZE/2;
    frontier(2,1) = MAP_SIZE/2;
    came_from = zeros(2, MAP_SIZE, MAP_SIZE);
    while (~found_path)
        for i = 1:1:3
            for j = 1:1:3
                if (i == 0) && (j == 0) % Don't check current pos
                    continue;
                end
                cur_x_ind = frontier(1, num_checked);
                cur_y_ind = frontier(2, num_checked);
                next_x_ind = cur_x_ind + (i-2);
                next_y_ind = cur_y_ind + (j-2);
                if (next_x_ind < 1) || (next_x_ind > MAP_SIZE) || (next_y_ind < 1) || (next_y_ind > MAP_SIZE)
                    continue; % Skip if out of map range
                end
                if( map( next_x_ind, next_y_ind ) > 0 ) % No obstacle in tile
                    frontier(1, num_checked+1) = next_x_ind;
                    frontier(2, num_checked+1) = next_y_ind;
                    came_from(1, next_x_ind, next_y_ind) = cur_x_ind;
                    came_from(2, next_x_ind, next_y_ind) = cur_y_ind;
                    num_checked = num_checked + 1;
                end
            end
        end
    end
    

    % Map visualization
    if(VISUALIZE_MAP)
        x_rng = [ map_origin(1) + TILE_SIZE/2, map_origin(1) + TILE_SIZE/2 + (TILE_SIZE * (MAP_SIZE-1))];
        y_rng = [ map_origin(2) + TILE_SIZE/2, map_origin(2) + TILE_SIZE/2 + (TILE_SIZE * (MAP_SIZE-1))];
        im = imagesc(x_rng, y_rng, map');
        im.AlphaData = VIS_MAP_ALPHA;
    end
end

