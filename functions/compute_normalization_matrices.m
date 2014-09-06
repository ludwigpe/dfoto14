% Method:   compute all normalization matrices.  
%           It is: point_norm = norm_matrix * point. The norm_points 
%           have centroid 0 and average distance = sqrt(2))
% 
%           Let N be the number of points and C the number of cameras.
%
% Input:    points2d is a 3xNxC array. Stores un-normalized homogeneous
%           coordinates for points in 2D. The data may have NaN values.
%        
% Output:   norm_mat is a 3x3xC array. Stores the normalization matrices
%           for all cameras, i.e. norm_mat(:,:,c) is the normalization
%           matrix for camera c.

function norm_mat = compute_normalization_matrices( points2d )

[~,~,C] = size(points2d);
centroids = zeros(3,C);
norm_mat = zeros(3,3,C);
% compute centroid for each set of points
for i = 1:C
    points = points2d(:,:,i);
    points = points(:,all(~isnan(points)));
    [~, N] = size(points);
    
    pc = sum(points,2)/N;
    
    dist = 0;
    for j = 1:N
        dist = dist + norm(points(:,j)-pc);
    end
    
    dc = dist/N;
    
    norm_mat(:,:,i) = (sqrt(2)/dc)*[1,0,0; 0,1,0; -pc(1), -pc(2), dc/sqrt(2)]';
   
end

norm_mat
end

%-------------------------
% TODO: FILL IN THIS PART