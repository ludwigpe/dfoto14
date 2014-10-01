% H = compute_rectification_matrix(points1, points2)
%
% Method: Determines the mapping H * points1 = points2
% 
% Input: points1, points2 of the form (4,n) 
%        n has to be at least 5
%
% Output:  H (4,4) matrix 
% 

function H = compute_rectification_matrix( points1, points2 )

    [~,N] = size(points1);
    xa = points1(1,:)';
    ya = points1(2,:)';
    za = points1(3,:)';
    wa = points1(4,:)';

    xb = points2(1,:)';
    yb = points2(2,:)';
    zb = points2(3,:)';

    r1 = [xa, ya, za, wa, zeros(N, 4), zeros(N, 4), -xa.*xb, -ya.*xb, -za.*xb, -wa.*xb];
    r2 = [zeros(N, 4), xa, ya, za, wa, zeros(N, 4), -xa.*yb, -ya.*yb, -za.*yb, -wa.*yb];
    r3 = [zeros(N, 4), zeros(N, 4), xa, ya, za, wa, -xa.*zb, -ya.*zb, -za.*zb, -wa.*zb]; 
    
    W = [r1;r2;r3];
    
    [U, S, V] = svd(W);
    h = V(:,end);
    H = vec2mat(h,4,4);

end
