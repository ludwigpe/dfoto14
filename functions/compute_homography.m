% H = compute_homography(points1, points2)
%
% Method: Determines the mapping H * points1 = points2
% 
% Input:  points1, points2 are of the form (3,n) with 
%         n is the number of points.
%         The points should be normalized for 
%         better performance.
% 
% Output: H 3x3 matrix 
%

function H = compute_homography( points1, points2 )

%-------------------------
% TODO: FILL IN THIS PART
% points2 are the reference points
% points1 are the other camera points
[~, n] = size(points1); % find out how many points
if any(isnan(points1(1,:)))
    idx = find(~isnan(points1));
    [r c] = ind2sub(size(points1),idx);
    start = c(1);
    stop = c(end);

    points1 = points1(:, all(~isnan(points1))); % remove nan columns
    points2 = points2(:, start:stop); % remove corresponding nan rows

    [~, n] = size(points1); % find out how many points
elseif any(isnan(points2(1,:)))
    idx = find(~isnan(points2));
    [r c] = ind2sub(size(points2),idx);
    start = c(1);
    stop = c(end);

    points2 = points2(:, all(~isnan(points2))); % remove nan columns
    points1 = points1(:, start:stop); % remove corresponding nan rows

    [~, n] = size(points2); % find out how many points
end

xref = points2(1,:)'; % make column vector of all xa
yref = points2(2,:)'; % make column vector of all ya

xc = points1(1,:)'; % make column vector of all xb
yc = points1(2,:)'; % make column vector of all yb

alpha = [xc, yc, ones(n,1), zeros(n,3), -xc.*xref, -yc.*xref,-xref];
beta  = [zeros(n,3), xc, yc, ones(n,1), -xc.*yref, -yc.*yref,-yref];

Q = [alpha;beta];

[U,S,V] = svd(Q);

h = V(:,end);
H = vec2mat(h,3);


end





