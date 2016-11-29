function [ desired_heading ] = pathfinder( current_map, robot_pos, target, SAVE_FILE )
%PATHFINDER Summary of this function goes here
%   Detailed explanation goes here
    
    % NOTE: Can't load variables from workspace into a function which uses
    %       nested functions. Avoid nested functions
    load( SAVE_FILE );
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                       MAP BUILDING                        %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
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
    dist_from_target = dist_from_target + (denom_check/10);
    map = (dist_from_target.^(-1)) * DIST_WEIGHT;
    
    % Give negative weighting to map tiles with obstacles (walls)
    % Not vectorized. For loop is slow, but easy
    % Check that indicies are within range
    temp = points_ind > 0;
    points_ind = points_ind .* temp;
    temp = points_ind <= MAP_SIZE;
    points_ind = points_ind .* temp;
    for j = 1:num_walls
        last_x = points_ind(1,1,j);
        last_y = points_ind(2,1,j);
        for k = 1:max_num_points
            x_ind = points_ind(1,k,j);
            y_ind = points_ind(2,k,j);
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
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                       PATHFINDING                         %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    % Find best path using custom variant of Dijkstra's algorithm
    % A* algorithm would be faster, but harder to implement
    % http://www.redblobgames.com/pathfinding/a-star/introduction.html
    num_added = 0;
    last_frontier(1,1) = ceil(MAP_SIZE/2);
    last_frontier(2,1) = ceil(MAP_SIZE/2);
    came_from = zeros(2, MAP_SIZE, MAP_SIZE);
    while ( last_frontier(1,1) ) % Run till frontier fully explored
        frontier = zeros(2,1); % Clear frontier variable
        [~, sz] = size( last_frontier );
        for i = 1:sz        
            cur_x_ind = last_frontier(1, i);
            cur_y_ind = last_frontier(2, i);
            for j = -1:1:1
                for k = -1:1:1
                    % Nested for loops allow movement in 4 directions (Forward,
                    % Back, Left, Right, Diagonal)
                    if (j == 0) && (k == 0) % Don't check current pos
                        continue;
                    end
                    next_x_ind = cur_x_ind + j;
                    next_y_ind = cur_y_ind + k;
                    if (next_x_ind < 1) || (next_x_ind > MAP_SIZE) || (next_y_ind < 1) || (next_y_ind > MAP_SIZE)
                        continue; % Skip if out of map range
                    end
                    % No 'came_from' data on tile means tile unvisited
                    old_tile = came_from(1, next_x_ind, next_y_ind);
                    if (map( next_x_ind, next_y_ind ) > 0) && (~old_tile) 
                        % No obstacle in tile and tile not visited before
                        % frontier grows every loop iteration (inefficient)
                        frontier(1, num_added+1) = next_x_ind;
                        frontier(2, num_added+1) = next_y_ind;
                        came_from(1, next_x_ind, next_y_ind) = cur_x_ind;
                        came_from(2, next_x_ind, next_y_ind) = cur_y_ind;
                        num_added = num_added + 1;
                    end
                end
            end
        end
        num_added = 0;
        
        
        %%%%%%%%%%%%%%%%%%%%%%%%%%% DEBUG
        temp_map = map;
        for i = 1:sz
            temp_map( last_frontier(1,i), last_frontier(2,i) ) = 0;
        end
        im = imagesc( temp_map' );
        %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
        
        last_frontier = frontier;
        
    end
    
    % Find highest value point within the allowed_map
    allowed_tile_mask(:,:) = came_from(1,:,:) > 0;
    allowed_map = map .* allowed_tile_mask;
    [~, temp_index] = max( allowed_map(:) );
    goal_index(2) = ceil( temp_index/MAP_SIZE );
    goal_index(1) = mod( temp_index - goal_index(2), MAP_SIZE ) + 1;
    goal_index
    
    % Create path by backtracking steps from goal to start
    found_path = 0;
    step = 1;
    path(:, step) = goal_index(:);
    while( ~found_path );
        step = step + 1;
        path(:, step) = came_from(:, path(1, step-1), path(2, step-1));
        if (path(1, step) == ceil(MAP_SIZE/2)) && (path(2, step) == ceil(MAP_SIZE/2))
            path = fliplr( path ); % Reverse order so path(1) is current pos
            found_path = 1;
        end
    end
    
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    %                       VISUALIZATION                       %
    %%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%%
    if(VISUALIZE_MAP)
        x_rng = [ map_origin(1) + TILE_SIZE/2, map_origin(1) + TILE_SIZE/2 + (TILE_SIZE * (MAP_SIZE-1))];
        y_rng = [ map_origin(2) + TILE_SIZE/2, map_origin(2) + TILE_SIZE/2 + (TILE_SIZE * (MAP_SIZE-1))];
        if(VISUALIZE_PATH)
            [~, sz] = size( path );
            for i = 1:sz
                map( path(1,i), path(2,i) ) = 0;
            end
        end
        im = imagesc(x_rng, y_rng, map');
        im.AlphaData = VIS_MAP_ALPHA;
    end
end

