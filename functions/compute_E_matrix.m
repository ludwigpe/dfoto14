% function E = compute_E_matrix( points1, points2, K1, K2 );
%
% Method:   Calculate the E matrix between two views from
%           point correspondences: points2^T * E * points1 = 0
%           we use the normalize 8-point algorithm and 
%           enforce the constraint that the three singular 
%           values are: a,a,0. The data will be normalized here. 
%           Finally we will check how good the epipolar constraints:
%           points2^T * E * points1 = 0 are fullfilled.
% 
%           Requires that the number of cameras is C=2.
% 
% Input:    points2d is a 3xNxC array storing the image points.
%
%           K is a 3x3xC array storing the internal calibration matrix for
%           each camera.
%
% Output:   E is a 3x3 matrix with the singular values (a,a,0).

function E = compute_E_matrix( points2d, K )

%------------------------------
% TODO: FILL IN THIS PART
IMPROVE_ACCURACY = true;

[~,N, C] = size(points2d);

points = zeros(3,N,C);
points_norm = points;
for c = 1:C
   % first normalization from original coordinates to
   % normalized camera coordinates.
   points(:,:,c) = K(:,:,c)\points2d(:,:,c); 
   
end
norm_mat = compute_normalization_matrices(points);
for c = 1:C
    % additional normalization
   points_norm(:,:,c) =  norm_mat(:,:,c)*points(:,:,c);
end

if IMPROVE_ACCURACY == true
    xa = points_norm(1,:,1)';
    ya = points_norm(2,:,1)';

    xb = points_norm(1,:,2)';
    yb = points_norm(2,:,2)';
else
    xa = points(1,:,1)';
    ya = points(2,:,1)';

    xb = points(1,:,2)';
    yb = points(2,:,2)';
end
W = [xb.*xa, xb.*ya, xb, yb.*xa, yb.*ya, yb, xa, ya, ones(N,1)];

[U,S,V] = svd(W);

if IMPROVE_ACCURACY == true
    f = V(:,end);
    F = vec2mat(f,3,3);
    for i = 1:N
        % check the epipolar constraint
        res = points_norm(:,i,2)'*F*points_norm(:,i,1);
        if(res > 10^-10)
            display('not good enough');
        end
    end
    % calculate the E-matrix from the F matrix
    E_norm = norm_mat(:,:,2)'*F*norm_mat(:,:,1);
    rankE = rank(E_norm)
   [U,S,V] = svd(E_norm);
   % calculate the correct E-matrix that fulfills the property in the
   % lecture notes.
   S_correct = (S(1,1) + S(2,2))/2;
   S = zeros(size(S));
   S(1,1) = S_correct;
   S(2,2) = S_correct;
   % Calculate the final E-matrix
   E = U*S*V';
   rank_correct = rank(E)
else
    e = V(:,end);
    E = vec2mat(e,3);
    for i = 1:N
        % check the epipolar constraint
        res = points(:,i,2)'*E*points(:,i,1);
        if(res > 10^-10)
            display('not good enough');
        end
    end
      rank2 = rank(E)
end

end









































































% 
% [~,n,~] = size(points2d);
% points = zeros(size(points2d));
% points_norm = zeros(size(points2d));
% W = zeros(n, 9);
% % first normalization
% pa = K(:,:,1)\points2d(:,:,1);
% pb = K(:,:,2)\points2d(:,:,2);
% 
% % compute normalization matrices from pa and pb
% norm_mat = zeros(3,3,2);
% norm_mat(:,:,1) = compute_normalization_matrices(pa);
% norm_mat(:,:,2) = compute_normalization_matrices(pb);
% 
% Na = norm_mat(:,:,1);
% Nb = norm_mat(:,:,2);
% 
% norm_points = zeros(3,n,2);
% norm_points(:,:,1) = norm_mat(:,:,1)*pa;
% norm_points(:,:,2) = norm_mat(:,:,2)*pb;
% 
% if IMPROVE_ACCURACY == true
%     xa = norm_points(1,:,1)';
%     xb = norm_points(1,:,2)';
%     
%     ya = norm_points(2,:,1)';
%     yb = norm_points(2,:,2)';
% 
% else
%     xa = pa(1,:)';
%     xb = pb(1,:)';
% 
%     ya = pa(2,:)';
%     yb = pb(2,:)';
% 
% end
% 
% W = [xb.*xa, xb.*ya, xb, yb.*xa,yb.*ya, yb, xa, ya, ones(n,1) ]
% 
% 
% % for i = 1:n  
% %     pa_orig = points2d(:,i,1);
% %     pb_orig = points2d(:,i,2);
% %     
% %     pa = K(:,:,1)\pa_orig;
% %     pb = K(:,:,2)\pb_orig;
% %     
% %     points(:,i,1) = pa;
% %     points(:,i,2) = pb;
% %    
% %     % End of first thing
% %     % Normalize for better accuracy
% %     pa_norm = Na*pa;
% %     pb_norm = Nb*pb;
% %      
% %     points_norm(:,i,1) = pa_norm;
% %     points_norm(:,i,2) = pb_norm;
% %     
% %     if IMPROVE_ACCURACY == true
% % 
% %         xa = pa_norm(1);
% %         ya = pa_norm(2);
% % 
% %         xb = pb_norm(1);
% %         yb = pb_norm(2);
% %     else 
% %         xa = pa(1);
% %         ya = pa(2);
% % 
% %         xb = pb(1);
% %         yb = pb(2);
% %     
% %     end
% %     
% %     W(i,:) = [xb*xa, xb*ya, xb, yb*xa, yb*ya, yb, xa, ya, 1];
% %   
% %     
% % end
% 
% [U,S,V] = svd(W);
% 
% if IMPROVE_ACCURACY == true
%     f = V(:,end);
%     F = vec2mat(f,3);
%     E = Nb'*F*Na;   
%     [U,S,V] = svd(E);
%     S
% else
%     e = V(:,end);
%     E = vec2mat(e,3);
%    [U,S,V] = svd(E);
%    S
% end
% 
% for i = 1:n
%     res = 1;
%     if IMPROVE_ACCURACY == true
%         pa = points_norm(:,i,1);
%         pb = points_norm(:,i,2)'; 
%         res = pb*F*pa;
%     else
%         pa = points(:,i,1);
%         pb = points(:,i,2)'; 
%         res = pb*E*pa;
%     end
%     
%     %res = pb*E*pa;
%     if(abs(res) > (10^-10))
%         display('Calculations are not accurate enough!');
%     end
% end
% 
% end


