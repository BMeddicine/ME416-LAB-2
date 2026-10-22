function [H, roundTripErr_mm] = findHomography(pointUV_px, pointUV_mm)
%FINDHOMOGRAPHY  Plane-to-pixel homography from N >= 4 point correspondences.
%   H = findHomography(pointUV_px, pointUV_mm) returns the 3x3 matrix H
%   such that [u'; v'; w'] = H * [U; V; 1], with u = u'/w' and v = v'/w'.
%
%   pointUV_px  Nx2 pixels, one point per row: [u, v]
%   pointUV_mm  Nx2 plane coordinates of the same points: [U, V]
%
%   [H, roundTripErr_mm] = findHomography(...) also maps the pixels back
%   through inv(H) and returns the Nx1 distance (mm) between each recovered
%   plane point and the input plane point. ~1e-12 for an exact 4-point fit;
%   with more points it is the least-squares residual.
%
%   Method: direct linear transform (DLT), from HW4 findHomography.m,
%   extended from exactly 4 to N points. Each correspondence gives two
%   linear equations in h11..h33; the solution is the right singular vector
%   of the 2Nx9 system with the smallest singular value. Both point sets are
%   first normalized (centroid at origin, mean distance sqrt(2)).

    arguments
        pointUV_px (:,2) double
        pointUV_mm (:,2) double
    end

    n = size(pointUV_px, 1);
    if n < 4 || size(pointUV_mm, 1) ~= n
        error("findHomography:badSize", ...
            "Need the same number (>= 4) of pixel and plane points; got %d and %d.", ...
            n, size(pointUV_mm, 1));
    end

    % Normalize both point sets for numerical conditioning
    [planeNorm, T_plane] = normalizePoints(pointUV_mm);
    [pixelNorm, T_pixel] = normalizePoints(pointUV_px);

    % Build the 2Nx9 DLT system A*h = 0, h = [h11 h12 h13 h21 h22 h23 h31 h32 h33]'
    %   u = (h11 U + h12 V + h13) / (h31 U + h32 V + h33)
    %   v = (h21 U + h22 V + h23) / (h31 U + h32 V + h33)
    A = zeros(2*n, 9);
    for k = 1:n
        U = planeNorm(k,1);  V = planeNorm(k,2);
        u = pixelNorm(k,1);  v = pixelNorm(k,2);
        A(2*k-1, :) = [U V 1  0 0 0  -u*U -u*V -u];
        A(2*k,   :) = [0 0 0  U V 1  -v*U -v*V -v];
    end

    % Degenerate if three or more of four points are collinear
    if rank(A) < 8
        error("findHomography:degenerate", ...
            "Correspondences are degenerate (points collinear).");
    end

    [~, ~, Vsvd] = svd(A, 0);
    Hnorm = reshape(Vsvd(:, end), 3, 3).';

    % Undo normalization and scale so h33 = 1
    H = T_pixel \ Hnorm * T_plane;
    H = H / H(3,3);

    % Round-trip check: pixels -> inv(H) -> plane
    if nargout > 1
        recoveredUV_mm = applyHomography(inv(H), pointUV_px);
        roundTripErr_mm = vecnorm(recoveredUV_mm - pointUV_mm, 2, 2);
    end
end

function [ptsNorm, T] = normalizePoints(pts)
% Similarity transform T that moves the centroid to the origin and scales
% the mean distance from it to sqrt(2). ptsNorm are the transformed points.
    centroid = mean(pts, 1);
    meanDist = mean(vecnorm(pts - centroid, 2, 2));
    s = sqrt(2) / meanDist;
    T = [s 0 -s*centroid(1);
         0 s -s*centroid(2);
         0 0  1];
    ptsH = T * [pts.'; ones(1, size(pts, 1))];
    ptsNorm = ptsH(1:2,:).';
end
