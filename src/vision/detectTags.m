function tags = detectTags(photoImg, tagFamily, intrinsics, hints)
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
%   tags = detectTags(..., hints) also tries to read tags the first pass
%   missed. hints is a struct array with .id and .cornerPx (4x2, 0-based
%   OpenCV pixels, e.g. from tools/check_stamp_holder.py). The stamp holder's
%   tags sit on a black pad with only a thin white line around them, so
%   readAprilTag can merge the tag's black border into the pad and miss it
%   (tag 3 on the real rack). For each hinted tag that is missing, the photo
%   is whited out outside the hinted outline and read again at full
%   resolution. Only readAprilTag's corners are used; OpenCV's outline sits
%   on the outer edge of the white line, ~5 px outside the tag's border, so
%   it is only good for locating the tag.
%
%   Requires the Computer Vision Toolbox (readAprilTag, undistortPoints).

    if nargin < 4
        hints = struct("id", {}, "cornerPx", {});
    end

    % readAprilTag returns IDs (Nx1) and corner locations (4x2xN, [x y])
    [tagIds, tagCorners_px] = readAprilTag(photoImg, tagFamily);
    tagIds = double(tagIds(:));
    tagCorners_px = double(tagCorners_px);

    for h = hints(:).'
        if ismember(h.id, tagIds)
            continue
        end
        corners_px = readMasked(photoImg, tagFamily, h);
        if ~isempty(corners_px)
            tagIds(end+1, 1) = h.id; %#ok<AGROW>
            tagCorners_px(:, :, end+1) = corners_px; %#ok<AGROW>
            fprintf("Tag %d: missed by the full-image pass, read inside its hinted outline.\n", h.id);
        end
    end

    tags = struct('id', {}, 'cornerUV_px', {}, 'cornerUndist_px', {}, 'centerUV_px', {});
    for k = 1:numel(tagIds)
        cornerUV_px = tagCorners_px(:, :, k);
        tags(k).id = tagIds(k);
        tags(k).cornerUV_px = cornerUV_px;
        tags(k).cornerUndist_px = undistortPoints(cornerUV_px, intrinsics);
        tags(k).centerUV_px = mean(cornerUV_px, 1);
    end

    if ~isempty(tags)
        [~, sortIdx] = sort([tags.id]);
        tags = tags(sortIdx);
    end
end

function corners_px = readMasked(photoImg, tagFamily, hint)
% Read one tag with everything outside its (slightly grown) hinted outline
% set to white. Tested on the real rack: 1.03 finds tag 3 and agrees with the
% full-image pass within 0.6 px on the tags that pass does find; larger
% margins let the black pad back in.
    outline = reshape(double(hint.cornerPx), 4, 2) + 1;          % 0-based -> 1-based
    centre = mean(outline, 1);
    corners_px = [];
    for grow = [1.03, 1.0, 1.06]
        poly = centre + grow * (outline - centre);
        mask = poly2mask(poly(:,1), poly(:,2), size(photoImg, 1), size(photoImg, 2));
        masked = photoImg;
        masked(repmat(~mask, 1, 1, size(photoImg, 3))) = intmax(class(photoImg));
        [ids, c] = readAprilTag(masked, tagFamily);
        k = find(double(ids) == hint.id, 1);
        if ~isempty(k)
            corners_px = double(c(:, :, k));
            return
        end
    end
end
