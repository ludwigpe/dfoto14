% function [cams, cam_centers] = reconstruct_stereo_cameras(E, K1, K2, data); 
%
% Method:   Calculate the first and second camera matrix. 
%           The second camera matrix is unique up to scale. 
%           The essential matrix and 
%           the internal camera matrices are known. Furthermore one 
%           point is needed in order solve the ambiguity in the 
%           second camera matrix.
%
%           Requires that the number of cameras is C=2.
%
% Input:    E is a 3x3 essential matrix with the singular values (a,a,0).
%
%           K is a 3x3xC array storing the internal calibration matrix for
%           each camera.
%
%           points2d is a 3xC matrix, storing an image point for each camera.
%
% Output:   cams is a 3x4x2 array, where cams(:,:,1) is the first and 
%           cams(:,:,2) is the second camera matrix.
%
%           cam_centers is a 4x2 array, where (:,1) is the first and (:,2) 
%           the second camera center.
%

function [cams, cam_centers] = reconstruct_stereo_cameras( E, K, points2d )

%------------------------------
% TODO: FILL IN THIS PART
Z = [0, 1, 0; -1, 0, 0; 0, 0, 0];
W = [0, -1, 0; 1, 0, 0; 0, 0, 1];
[U,S,V] = svd(E);
t = V(:,end)

cam_centers = zeros(4,2);
cam_centers(4,:) = 1;
cam_centers(1:3,2) = -t;

R1 = U*W*(V');
R2 = U*(W')*(V');

if det(R1) == -1
    R1 = R1*-1;
end
if det(R2) == -1
    R2 = R2*-1;
end


Ma = K(:,:,1)*eye(3,4);
Mb = zeros(3,4,4);

Ka = K(:,:,1);
Kb = K(:,:,2);

It1 = eye(3,4);
It1(:,4) = t;

It2 = eye(3,4);
It2(:,4) = -t;

% all alternatives of camera b
Mb(:,:,1) = Kb*R1*It1;
Mb(:,:,2) = Kb*R1*It2;
Mb(:,:,3) = Kb*R2*It1;
Mb(:,:,4) = Kb*R2*It2;

cameras = zeros(3,4,2);
cameras(:,:,1) = Ma;

points3d = zeros(4,4);
for i = 1:4
   cameras(:,:,2) = Mb(:,:,i);
   points3d(:,i) = reconstruct_point_cloud(cameras, points2d(:,1,:));
end

% find the correct pair of cameras where the point is in front of both
% cameras
for i = 1:4
   p = points3d(:,i);
   m1 = inv(Ka)*Ma;
   m2 = inv(Kb)*Mb(:,:,i);
   point_m1 = m1*p;
   point_m2 = m2*p;
   
   z1 = point_m1(3);
   z2 = point_m2(3);
   
   if(z1 >= 0 && z2 >=0)
      cams = zeros(3,4,2);
      cams(:,:,1) = Ma;
      cams(:,:,2) = Mb(:,:,i);
   end
   
end


