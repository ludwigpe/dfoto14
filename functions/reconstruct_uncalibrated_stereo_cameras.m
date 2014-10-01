% function [cams, cam_centers] = reconstruct_uncalibrated_stereo_cameras(F); 
%
% Method: Calculate the first and second uncalibrated camera matrix
%         from the F-matrix. 
% 
% Input:  F - Fundamental matrix with the last singular value 0 
%
% Output:   cams is a 3x4x2 array, where cams(:,:,1) is the first and 
%           cams(:,:,2) is the second camera matrix.
%
%           cam_centers is a 4x2 array, where (:,1) is the first and (:,2) 
%           the second camera center.

function [cams, cam_centers] = reconstruct_uncalibrated_stereo_cameras( F )

[U,S,V] = svd(F');
h = V(:,end);

% creat the antisymetric matrix S
S = [0, -1, 1; 1, 0 , -1; -1, 1, 0];

cams(:,:,1) = eye(3,4);
Mb(:,1:3) = S*F;
Mb(:,end+1) = h;
cams(:,:,2) = Mb;

for c = 1:2
    [U,S,V] = svd(cams(:,:,c));
    cam_centers(:,c) = V(:,end);
end

end
