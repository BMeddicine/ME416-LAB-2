function pose = estimateTablePose(refUndist_px, refUVW_mm, K)
%ESTIMATETABLEPOSE Camera pose relative to the world (base) frame from tags.
%   pose = estimateTablePose(refUndist_px, refUVW_mm, K)
%     refUndist_px  Nx2 undistorted pixels of reference tag corners (N >= 4)
%     refUVW_mm     Nx3 world coordinates of the same corners; all must share
%                   one height W = h0 (they lie on one plane)
%     K             3x3 intrinsic matrix (MATLAB convention, 1-based pixels)
%
%   Returns pose with fields
%     R, t            camera coordinates of a world point X: R*X + t
%     K
%     H               table (W = 0) to undistorted pixel homography, K*[r1 r2 t]
%     cameraCenter_mm camera position in the world frame
%     reprojRms_px    RMS reprojection error of the reference corners
%     reprojRms_mm    the same error measured on the reference plane
%     nRefPoints
%
%   Method:
%     1. H0 = findHomography on the plane W = h0.
%     2. Split H0 with K: [r1 r2 t0] = lambda * inv(K) * H0, r3 = r1 x r2,
%        with lambda's sign chosen so the plane is in front of the camera,
%        then the nearest true rotation via SVD. Shift t to the W = 0 plane.
%     3. Gauss-Newton on the 6 pose parameters to minimize pixel
%        reprojection error (step 2 only projects H0 onto a rotation, which
%        is not the best fit when the corners are noisy).
%   Handedness check: the camera must be above the table (W > 0). If it
%   comes out below, the corner winding or yaw convention is mirrored.

    arguments
        refUndist_px (:,2) double
        refUVW_mm    (:,3) double
        K            (3,3) double
    end

    h0 = refUVW_mm(1,3);
    if any(abs(refUVW_mm(:,3) - h0) > 1e-9)
        error("estimateTablePose:notPlanar", "All reference corners must be at the same height.");
    end

    % 1-2. Homography on the reference plane, then split with K
    H0 = findHomography(refUndist_px, refUVW_mm(:,1:2));
    M = K \ H0;
    lambda = 2 / (norm(M(:,1)) + norm(M(:,2)));
    if M(3,3) * lambda < 0
        lambda = -lambda;          % plane must be in front of the camera (z > 0)
    end
    r1 = lambda * M(:,1);
    r2 = lambda * M(:,2);
    t0 = lambda * M(:,3);
    [Us, ~, Vs] = svd([r1 r2 cross(r1, r2)]);
    R = Us * diag([1 1 det(Us * Vs.')]) * Vs.';
    t = t0 - h0 * R(:,3);          % t0 is for the plane W = h0; t is for W = 0

    % 3. Refine by minimizing reprojection error
    [R, t] = refinePose(R, t, refUndist_px, refUVW_mm, K);

    cameraCenter_mm = (-R.' * t).';
    if cameraCenter_mm(3) <= 0
        error("estimateTablePose:mirrored", ...
            ['The camera came out below the table (W = %.0f mm). The tag corner order is ' ...
             'mirrored: flip tagCornerWinding in config/layout.json (cw <-> ccw), or check ' ...
             'that the base tag yaw is measured counterclockwise from +U.'], cameraCenter_mm(3));
    end

    pose.R = R;
    pose.t = t;
    pose.K = K;
    pose.H = K * [R(:,1) R(:,2) t];
    pose.H = pose.H / pose.H(3,3);
    pose.cameraCenter_mm = cameraCenter_mm;

    reproj_px = projectWorldToPixel(pose, refUVW_mm);
    pose.reprojRms_px = sqrt(mean(sum((reproj_px - refUndist_px).^2, 2)));
    recovered_mm = applyHomography(inv(homographyAtHeight(pose, h0)), refUndist_px);
    pose.reprojRms_mm = sqrt(mean(sum((recovered_mm - refUVW_mm(:,1:2)).^2, 2)));
    pose.nRefPoints = size(refUVW_mm, 1);
end

function [R, t] = refinePose(R, t, px, X, K)
% Gauss-Newton on p = [rotation vector (3); translation (3)], with the
% rotation applied as a small update on the left of the current R.
    maxIterations = 30;
    for iter = 1:maxIterations
        r0 = residual(R, t, px, X, K);
        J = zeros(numel(r0), 6);
        step = 1e-6;
        for j = 1:6
            dp = zeros(6, 1);
            dp(j) = step;
            [Rj, tj] = update(R, t, dp);
            J(:, j) = (residual(Rj, tj, px, X, K) - r0) / step;
        end
        dp = -(J \ r0);
        [R, t] = update(R, t, dp);
        if norm(dp) < 1e-10
            break
        end
    end
end

function [R, t] = update(R, t, dp)
    R = rodrigues(dp(1:3)) * R;
    t = t + dp(4:6);
end

function r = residual(R, t, px, X, K)
    xc = R * X.' + t;
    uvh = K * (xc ./ xc(3,:));
    r = reshape((uvh(1:2,:).' - px).', [], 1);
end

function Rv = rodrigues(w)
% Rotation matrix for rotation vector w (axis * angle in radians).
    theta = norm(w);
    if theta < 1e-15
        Rv = eye(3);
        return
    end
    k = w / theta;
    Kx = [0 -k(3) k(2); k(3) 0 -k(1); -k(2) k(1) 0];
    Rv = eye(3) + sin(theta) * Kx + (1 - cos(theta)) * (Kx * Kx);
end
