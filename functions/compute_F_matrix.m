% function F = compute_F_matrix(points1, points2);
%
% Method:   Calculate the F matrix between two views from
%           point correspondences: points2^T * F * points1 = 0
%           We use the normalize 8-point algorithm and 
%           enforce the constraint that the three singular 
%           values are: a,b,0. The data will be normalized here. 
%           Finally we will check how good the epipolar constraints:
%           points2^T * F * points1 = 0 are fullfilled.
% 
%           Requires that the number of cameras is C=2.
% 
% Input:    points2d is a 3xNxC array storing the image points.
%
% Output:   F is a 3x3 matrix where the last singular value is zero.

function F = compute_F_matrix( points2d )

[~,N, C] = size(points2d);
points_norm = zeros(size(points2d));
% compute normalization matrices
norm_mat = compute_normalization_matrices(points2d);
for c = 1:C
    points_norm(:,:,c) = norm_mat(:,:,c)*points2d(:,:,c);
end

xa = points_norm(1,:,1)';
ya = points_norm(2,:,1)';

xb = points_norm(1,:,2)';
yb = points_norm(2,:,2)';

W = [xb.*xa, xb.*ya, xb, yb.*xa, yb.*ya, yb, xa, ya, ones(N,1)];

[U,S,V] = svd(W);

f = V(:,end);
F = vec2mat(f,3,3);
F = norm_mat(:,:,2)'*F*norm_mat(:,:,1);
rankF = rank(F);
if rankF > 2
    % F should be rank 2 so we force it
    [U,S,V] = svd(F);
    F = U(:,1:2)*S(1:2,1:2)*V(:,1:2)';
end

end