function cornerUVW_mm = tagCornersWorld(centerUV_mm, yaw_deg, tagSize_mm, height_mm, winding)
%TAGCORNERSWORLD World coordinates of a tag's 4 corners, in detector order.
%   cornerUVW_mm = tagCornersWorld(centerUV_mm, yaw_deg, tagSize_mm, height_mm, winding)
%   returns a 4x3 matrix, row i = [U, V, W] of corner i-1, in the same order
%   readAprilTag reports corners.
%
%   Tag frame: x along corner 0 -> corner 1, y 90 deg counterclockwise from
%   x seen from above. yaw_deg is the direction of x in the world frame.
%   winding is the order corners 0 -> 1 -> 2 -> 3 run, seen from above:
%     "cw"  - clockwise, so corners 2 and 3 are on the -y side
%     "ccw" - counterclockwise, corners 2 and 3 on the +y side
%   Seen from above is the same as seen on screen in an overhead photo, so
%   hw4B1.m's "order 0->1->2->3 runs ... on screen" line gives the value.
%   readAprilTag gives "ccw" (checked on the HW4 tag photo, all 4 tags).
%   If it is set wrong, estimateTablePose fails its handedness check.

    a = tagSize_mm / 2;
    switch winding
        case "cw"
            cornerTag_mm = [-a  a;  a  a;  a -a; -a -a];
        case "ccw"
            cornerTag_mm = [-a -a;  a -a;  a  a; -a  a];
        otherwise
            error("tagCornersWorld:winding", "winding must be ""cw"" or ""ccw"", got ""%s"".", winding);
    end

    rot = [cosd(yaw_deg) -sind(yaw_deg); sind(yaw_deg) cosd(yaw_deg)];
    cornerUV_mm = (rot * cornerTag_mm.').' + centerUV_mm(:).';
    cornerUVW_mm = [cornerUV_mm, repmat(height_mm, 4, 1)];
end
