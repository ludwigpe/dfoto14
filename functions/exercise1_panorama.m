

% Method:   Generate one image out of multiple images. All images are from
%           a camera with the same (!) center of projection. All the images 
%           are registered to one reference view.

clear all                   % Remove all old variables
close all                   % Close all figures
clc                         % Clear the command window
addpath( genpath( '../' ) );% Add paths to all subdirectories of the parent directory

NORMALIZE           = true;
LOAD_DATA           = true;
REFERENCE_VIEW      = 3;
CAMERAS             = 3;
image_names_file    = '../images/names_images_kth.txt';
name_panorama       = '../images/panorama_image.jpg';
points2d_file       = '../data/data_kth.mat';
    
[images, name_loaded_images] = load_images_grey( image_names_file, CAMERAS );

% Load the clicked points if they have been saved,
% or click some new points:
if LOAD_DATA
    load( points2d_file );
else
    points2d = click_multi_view( images ); %, C, data, 0 ); % for clicking and displaying data
    save( points2d_file, 'points2d' );
end


%% Compute homographies
% Determine all homographies to a reference view. We have:
% point in REFERENCE_VIEW = homographies(:,:,c) * point in image c.
% Remember, you have to set homographies{REFERENCE_VIEW} as well.
homographies = zeros(3,3,CAMERAS); 
homographies_norm = zeros(3,3,CAMERAS); 

points2d_norm = zeros(size(points2d));

% compute Normalization matrices
norm_mat = compute_normalization_matrices(points2d);

for c = 1:CAMERAS
    % compute normalized points for camera c
    points2d_norm(:,:,c) = norm_mat(:,:,c)*points2d(:,:,c);
    compute_normalization_matrices(points2d_norm(:,:,c))
end
refPoints_norm = points2d_norm(:,:,REFERENCE_VIEW);
norm_ref = norm_mat(:,:,REFERENCE_VIEW); % Normalization matrix for reference camera

for c = 1:CAMERAS
    if NORMALIZE
        points_norm = norm_mat(:,:,c)*points2d(:,:,c);
        h_norm = compute_homography(points_norm, refPoints_norm);
        Nc = norm_mat(:,:,c);
        H = norm_ref\h_norm*Nc;
        homographies(:,:,c) = H;
    else
        points = points2d(:,:,c);
        refPoints = points2d(:,:,REFERENCE_VIEW);
        homographies(:,:,c) = compute_homography(points, refPoints);
    end
end

%-------------------------
% TODO: FILL IN THIS PART



for c = 1:CAMERAS
    
    [error_mean error_max] = check_error_homographies( ...
      homographies(:,:,c), points2d(:,:,c), points2d(:,:,REFERENCE_VIEW) );
 
    fprintf( 'Between view %d and reff. view; ', c );
    fprintf( 'average error: %5.2f; maximum error: %5.2f \n', error_mean, error_max );
end


%% Generate, draw and save panorama

panorama_image = generate_panorama( images, homographies );

figure;  
show_image_grey( panorama_image );
save_image_grey( name_panorama, panorama_image );
