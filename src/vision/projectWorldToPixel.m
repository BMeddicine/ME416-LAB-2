function uv_px = projectWorldToPixel(pose, UVW_mm, radialDistortion)
%PROJECTWORLDTOPIXEL Project world points into the image.
%   uv_px = projectWorldToPixel(pose, UVW_mm) gives undistorted pixels
%   (what the homography math works in) for Nx3 world points.
%   uv_px = projectWorldToPixel(pose, UVW_mm, radialDistortion) applies
%   MATLAB's radial model [k1 k2 ...] so the pixels land on the raw photo
%   (used for drawing).

    xc = pose.R * UVW_mm.' + pose.t;
    xn = xc(1:2,:) ./ xc(3,:);
    if nargin > 2 && ~isempty(radialDistortion)
        r2 = sum(xn.^2, 1);
        scale = ones(1, size(xn, 2));
        for k = 1:numel(radialDistortion)
            scale = scale + radialDistortion(k) * r2.^k;
        end
        xn = xn .* scale;
    end
    uvh = pose.K * [xn; ones(1, size(xn, 2))];
    uv_px = uvh(1:2,:).';
end
