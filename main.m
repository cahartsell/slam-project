function [ ] = main( )
%MAIN Summary of this function goes here
%   Detailed explanation goes here

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
    MAP_SIZE            = 11; %Should be odd number
    TILE_SIZE           = 5;
    
    % Save macros to file so all functions can access. Overwrite old file
    save( SAVE_FILE );
    
    generateEnvironment( SAVE_FILE );
    
    load( SAVE_FILE );
    
    pathfinder( wall_map, robot_start, target_pos, SAVE_FILE );


end
