% function model = reconstruct_point_cloud(cam, data)
%
% Method:   Determines the 3D model points by triangulation
%           of a stereo camera system. We assume that the data 
%           is already normalized 
% 
%           Requires that the number of cameras is C=2.
%           Let N be the number of points.
%
% Input:    points2d is a 3xNxC array, storing all image points.
%
%           cameras is a 3x4xC array, where cameras(:,:,1) is the first and 
%           cameras(:,:,2) is the second camera matrix.
% 
% Output:   points3d 4xN matrix of all 3d points.


function points3d = reconstruct_point_cloud( cameras, points2d )

%------------------------------
% TODO: FILL IN THIS PART

C = 2;
[~,N,~] = size(points2d);
points3d = zeros(4,N);
for i = 1:N
    M = [];
    for j = 1:C
        x = points2d(1,i,j);
        y = points2d(2,i,j);
        
        m1 = cameras(1,:,j);
        m2 = cameras(2,:,j);
        m3 = cameras(3,:,j);
        r1 = xa*ma3-(1*m1)
        r2 = ya*ma3-(1*m2)
        M = [M;r1;r2];
    end
   [U,S,V] = svd(M)
   p3d = V(:,end);
   points3d(:,i) = p3d;
end



