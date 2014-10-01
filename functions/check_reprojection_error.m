% function [error_average, error_max] = check_reprojection_error(data, cam, model)
%
% Method:   Evaluates average and maximum error 
%           between the reprojected image points (cam*model) and the 
%           given image points (data), i.e. data = cam * model 
%
%           We define the error as the Euclidean distance in 2D.
%
%           Requires that the number of cameras is C=2.
%           Let N be the number of points.
%
% Input:    points2d is a 3xNxC array, storing all image points.
%
%           cameras is a 3x4xC array, where cams(:,:,1) is the first and 
%           cameras(:,:,2) is the second camera matrix.
%
%           point3d 4xN matrix of all 3d points.
%       
% Output:   
%           The average error (error_average) and maximum error (error_max)
%      

function [error_average, error_max] = check_reprojection_error( points2d, cameras, points3d )


[~,N] = size(points3d);
errors = [];
% calc_points = zeros(3,N,2);
% calc_points(:,:,1) = cameras(:,:,1)*points3d;
% calc_points(:,:,2) = cameras(:,:,2)*points3d;
% 
% cam_1 = cameras(:,:,1)*points3d;
% cam_2 = cameras(:,:,2)*points3d;
% cam_1_cartesian = homogeneous_to_cartesian(cam_1);
% cam_2_cartesian = homogeneous_to_cartesian(cam_2);
% 
% real_cartesian_1 = homogeneous_to_cartesian(points2d(:,:,1));
% real_cartesian_2 = homogeneous_to_cartesian(points2d(:,:,2));
% 
% diff_cam_1 = real_cartesian_1 - cam_1_cartesian;
% diff_cam_2 = real_cartesian_2 - cam_2_cartesian;
% error_cam1 = sqrt(sum(abs(diff_cam_1).^2,1)) %# The two-norm of each column
% error_cam2 = sqrt(sum(abs(diff_cam_2).^2,1)) %# The two-norm of each column
% errors = [error_cam1, error_cam2];

for c = 1:2
   % use our calculated cameras to project 3dpoints to 2d
   projected_points = cameras(:,:,c)*points3d;
   
   % transform points from homogeneous to cartesian
   projected_cartesian_points =  homogeneous_to_cartesian(projected_points);
   
   % transform the stored image point to cartesian
   img_cartesian = homogeneous_to_cartesian(points2d(:,:,c));
   
   % parwise difference between points
   diff = img_cartesian - projected_cartesian_points;
   
   % calculate the norm of each diff-vector
   error = sqrt(sum(abs(diff).^2,1)); %# The two-norm of each column
   errors = [errors,error]
   
end


error_average = mean(errors);
error_max = max(errors);