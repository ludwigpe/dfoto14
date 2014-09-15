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

[~,n,~] = size(points2d);
points = zeros(size(points2d));
points_norm = zeros(size(points2d));
norm_mat = compute_normalization_matrices(points2d);
Na = norm_mat(:,:,1);
Nb = norm_mat(:,:,2);

W = zeros(n, 9);
for i = 1:n  
    pa_orig = points2d(:,i,1);
    pb_orig = points2d(:,i,2);
    
    pa = K(:,:,1)\pa_orig;
    pb = K(:,:,2)\pb_orig;

    points(:,i,1) = pa;
    points(:,i,2) = pb;
   
    % End of first thing
    % Normalize for better accuracy
    pa_norm = Na*pa;
    pb_norm = Nb*pb;
     
    points_norm(:,i,1) = pa_norm;
    points_norm(:,i,2) = pb_norm;
    
    if(IMPROVE_ACCURACY)
        xa = pa_norm(1);
        ya = pa_norm(2);

        xb = pb_norm(1);
        yb = pb_norm(2);
    else
            
        xa = pa(1);
        ya = pa(2);

        xb = pb(1);
        yb = pb(2);
    
    end
    
    W(i,:) = [xb*xa, xb*ya, xb, yb*xa, yb*ya, yb, xa, ya, 1];
  
    
end
[U,S,V] = svd(W);

if(IMPROVE_ACCURACY)
    f = V(:,end);
    F = vec2mat(f,3);
    E = Nb'*F*Na;

else
    e = V(:,end);
    E = vec2mat(e,3);
end



for i = 1:n
    pa = points(:,i,1);
    pb = points(:,i,2)'; 
    res = pb*E*pa;
    if(res > (1/10^-10))
        display('Calculations are not accurate enough!');
    end
    
end

end


