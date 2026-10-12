function tags = detectTags(photoImg, tagFamily, intrinsics)
%DETECTTAGS Detect AprilTags in one photograph.
%   tags = detectTags(photoImg, tagFamily, intrinsics) returns a struct
%   array with one element per detected tag, sorted by ID:
%       tags(k).id               - tag ID (integer)
%       tags(k).cornerUV_px      - 4x2, row i = [u, v] of corner i-1, as
%                                  readAprilTag returns them (raw photo,
%                                  MATLAB 1-based pixels)
%       tags(k).cornerUndist_px  - 4x2, the same corners with lens
%                                  distortion removed (same K)
%       tags(k).centerUV_px      - 1x2 mean of the four raw corners
%   Based on HW4 detectTags.m; adds the undistorted corners, which are what
%   the homography math uses.
%
%   Requires the Computer Vision Toolbox (readAprilTag, undistortPoints).

    % readAprilTag returns IDs (Nx1) and corner locations (4x2xN, [x y])
    [tagIds, tagCorners_px] = readAprilTag(photoImg, tagFamily);

    tags = struct('id', {}, 'cornerUV_px', {}, 'cornerUndist_px', {}, 'centerUV_px', {});
    for k = 1:numel(tagIds)
        cornerUV_px = double(tagCorners_px(:, :, k));
        tags(k).id = double(tagIds(k));
        tags(k).cornerUV_px = cornerUV_px;
        tags(k).cornerUndist_px = undistortPoints(cornerUV_px, intrinsics);
        tags(k).centerUV_px = mean(cornerUV_px, 1);
    end

    if ~isempty(tags)
        [~, sortIdx] = sort([tags.id]);
        tags = tags(sortIdx);
    end
end
