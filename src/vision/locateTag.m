function located = locateTag(tag, pose, height_mm, tagSize_mm, sideTolerance)
%LOCATETAG World position and yaw of one detected tag lying at a known height.
%   located = locateTag(tag, pose, height_mm, tagSize_mm, sideTolerance)
%   maps the tag's undistorted corners back through H_h (the homography of
%   the plane W = height_mm) and returns
%     id, height_mm
%     centerUV_mm   centre (mean of the 4 corners on the plane)
%     yaw_deg       direction of corner 0 -> corner 1, from +U toward +V
%     cornerUV_mm   4x2 corners on the plane
%     cornerPx      4x2 raw corner pixels (for drawing)
%     centerPx      1x2 raw centre pixel (for the Problem 2 table)
%     meanSide_mm, sideError (fraction of tagSize_mm), sideOk
%   A side length off by more than sideTolerance usually means height_mm is
%   wrong for this tag (parallax scales its apparent size) or a bad detection.

    Hh = homographyAtHeight(pose, height_mm);
    cornerUV_mm = applyHomography(inv(Hh), tag.cornerUndist_px);
    edge01_mm = cornerUV_mm(2,:) - cornerUV_mm(1,:);
    sideLen_mm = vecnorm(cornerUV_mm - circshift(cornerUV_mm, -1), 2, 2);

    located.id = tag.id;
    located.height_mm = height_mm;
    located.centerUV_mm = mean(cornerUV_mm, 1);
    located.yaw_deg = atan2d(edge01_mm(2), edge01_mm(1));
    located.cornerUV_mm = cornerUV_mm;
    located.cornerPx = tag.cornerUV_px;
    located.centerPx = tag.centerUV_px;
    located.meanSide_mm = mean(sideLen_mm);
    located.sideError = located.meanSide_mm / tagSize_mm - 1;
    located.sideOk = abs(located.sideError) <= sideTolerance;
end
