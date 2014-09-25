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


[~,N] = size(points3d)
errors = [];
for c = 1:2
    cam = cameras(:,:,c);
    
    for i = 1:N
       real_point2d = points2d(:,i,c);
       calc_point2d = cam*points3d(:,N);
       real_point2d = homogeneous_to_cartesian(real_point2d);
       calc_point2d = homogeneous_to_cartesian(calc_point2d);
       error = norm(real_point2d-calc_point2d);
       errors = [errors,error];
    end
end

error_average = mean(errors);
error_max = max(errors);