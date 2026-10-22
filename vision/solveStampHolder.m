function result = solveStampHolder(tags, layout, K, holder)
%SOLVESTAMPHOLDER Base tag + the holder's two tags -> all 26 stamp tops in the base frame.
%   result = solveStampHolder(tags, layout, K, holder)
%     tags    struct array from detectTags
%     layout  from loadLayout (tagSize_mm, tagCornerWinding, sideLengthTolerance, baseTag)
%     K       3x3 intrinsic matrix
%     holder  from loadStampHolder (config/stamp_holder.json)
%
%   Method:
%     1. Camera pose from the base tag (as solveTagTarget).
%     2. Holder tag centres. The tags sit on pads tilted ~9 deg, so their
%        corners are at different heights. The crossing of the image
%        diagonals is the exact image of the tag centre (perspective keeps
%        lines and incidence), and the centre's height is known, so it is
%        mapped back through H_h for that one height. Only the centres are
%        used, so it does not matter how each tag is rotated on its pad.
%     3. Holder frame: origin = midpoint of the two centres (on the plate),
%        +x = left tag -> right tag, +y 90 deg CCW from +x seen from above.
%     4. Each stamp = holder pose applied to its CAD offset; top = seat + stamp height.
%
%   Fields:
%     pose, baseCheck, warnings    as solveTagTarget
%     tags      base, left and right holder tags (for drawScene), then any
%               other detected tags (role "other", located at
%               holder.otherTagHeight_mm, default 0), e.g. tag 2 by the ink pad
%     holder    .origin_UV_mm, .yaw_deg (direction of holder +x),
%               .leftTag_UV_mm, .rightTag_UV_mm, .baseline_mm, .baselineError_mm
%     stamps(k) .name, .row, .top_UVW_mm, .seat_UVW_mm, .rho_mm, .phi_deg
%               (of the top), .radialDir_deg (well's long axis, pointing
%               away from the arc centre), .rimCorners_UVW_mm (4x3, the
%               well opening, for drawing)

    warnings = strings(0, 1);
    tagSize_mm = requireValue(layout.tagSize_mm, "tagSize_mm");

    % ---- 1. Pose from the base tag
    base = layout.baseTag;
    baseId = requireValue(base.id, "baseTag.id");
    baseCenter_mm = requireValue(base.centerUV_mm, "baseTag.centerUV_mm");
    baseYaw_deg = requireValue(base.yaw_deg, "baseTag.yaw_deg");
    baseHeight_mm = requireValue(base.height_mm, "baseTag.height_mm");

    baseTag = findTag(tags, baseId, "base tag (baseTag.id)");
    leftTag = findTag(tags, holder.leftTagId, "left holder tag (leftTagId)");
    rightTag = findTag(tags, holder.rightTagId, "right holder tag (rightTagId)");

    refUVW = tagCornersWorld(baseCenter_mm, baseYaw_deg, tagSize_mm, baseHeight_mm, ...
        layout.tagCornerWinding);
    pose = estimateTablePose(baseTag.cornerUndist_px, refUVW, K);
    pose.refTagIds = baseId;

    baseLoc = locateTag(baseTag, pose, baseHeight_mm, tagSize_mm, layout.sideLengthTolerance);
    baseLoc.role = "base";
    result.baseCheck.centerError_mm = norm(baseLoc.centerUV_mm - baseCenter_mm);
    result.baseCheck.yawError_deg = mod(baseLoc.yaw_deg - baseYaw_deg + 180, 360) - 180;

    % ---- 2. Holder tag centres at their height
    h = holder.tagCenterHeight_mm;
    Hh = homographyAtHeight(pose, h);
    leftLoc = locateTag(leftTag, pose, h, tagSize_mm, layout.sideLengthTolerance);
    rightLoc = locateTag(rightTag, pose, h, tagSize_mm, layout.sideLengthTolerance);
    leftLoc.centerUV_mm = applyHomography(inv(Hh), diagonalCrossing(leftTag.cornerUndist_px));
    rightLoc.centerUV_mm = applyHomography(inv(Hh), diagonalCrossing(rightTag.cornerUndist_px));
    leftLoc.role = "holder L";
    rightLoc.role = "holder R";
    for loc = [leftLoc, rightLoc]
        if ~loc.sideOk
            warnings(end+1) = sprintf(['Holder tag %d: side %.1f mm vs %.1f mm expected (%+.1f%%). ' ...
                'The ~9 deg tilt alone can account for about 2%%; more means a bad detection or a ' ...
                'different printed tag size. The stamp positions only use the tag centres.'], loc.id, loc.meanSide_mm, tagSize_mm, 100 * loc.sideError); %#ok<AGROW>
        end
    end

    % ---- 2b. Any other tags (e.g. tag 2 by the ink pad), at holder.otherTagHeight_mm
    otherHeight_mm = 0;
    if isfield(holder, "otherTagHeight_mm")
        otherHeight_mm = holder.otherTagHeight_mm;
    end
    others = tags(~ismember([tags.id], [baseId, holder.leftTagId, holder.rightTagId]));
    otherLocs = [];
    for tag = others
        loc = locateTag(tag, pose, otherHeight_mm, tagSize_mm, layout.sideLengthTolerance);
        loc.role = "other";
        if ~loc.sideOk
            warnings(end+1) = sprintf(['Tag %d: side %.1f mm vs %.1f mm expected (%+.1f%%). Is it ' ...
                'at otherTagHeight_mm = %g mm (config/stamp_holder.json)?'], loc.id, ...
                loc.meanSide_mm, tagSize_mm, 100 * loc.sideError, otherHeight_mm); %#ok<AGROW>
        end
        otherLocs = [otherLocs, loc]; %#ok<AGROW>
    end

    % ---- 3. Holder frame
    d = rightLoc.centerUV_mm - leftLoc.centerUV_mm;
    baseline_mm = norm(d);
    xHat = d / baseline_mm;
    yHat = [-xHat(2), xHat(1)];
    origin = (leftLoc.centerUV_mm + rightLoc.centerUV_mm) / 2;
    yaw_deg = atan2d(xHat(2), xHat(1));
    toWorld = @(xy) origin + xy(:,1) * xHat + xy(:,2) * yHat;

    hp.origin_UV_mm = origin;
    hp.yaw_deg = yaw_deg;
    hp.leftTag_UV_mm = leftLoc.centerUV_mm;
    hp.rightTag_UV_mm = rightLoc.centerUV_mm;
    hp.baseline_mm = baseline_mm;
    hp.baselineError_mm = baseline_mm - holder.tagBaseline_mm;
    if abs(hp.baselineError_mm) > holder.baselineTolerance_mm
        warnings(end+1) = sprintf(['Holder tags are %.1f mm apart, CAD says %.1f mm (%+.1f mm). ' ...
            'Check the tag IDs (left = %d, right = %d), the base tag layout and the tag height.'], ...
            baseline_mm, holder.tagBaseline_mm, hp.baselineError_mm, holder.leftTagId, holder.rightTagId);
    end

    % ---- 4. Stamps (the holder stands on the plate, W = 0)
    half = holder.rimSize_mm / 2;          % [tangential, radial]
    stamps = struct("name", {}, "row", {}, "top_UVW_mm", {}, "seat_UVW_mm", {}, "rho_mm", {}, ...
        "phi_deg", {}, "radialDir_deg", {}, "rimCorners_UVW_mm", {});
    for k = 1:numel(holder.stamps)
        c = holder.stamps(k);
        seatUV = toWorld(c.seatXYZ_mm(1:2));
        th = c.radialYaw_deg;
        radial = [sind(th), cosd(th)];        % in H
        tangential = [cosd(th), -sind(th)];
        rimH = c.seatXYZ_mm(1:2) + ([-1 -1; 1 -1; 1 1; -1 1] .* half) * [tangential; radial];
        s.name = c.name;
        s.row = c.row;
        s.top_UVW_mm = [seatUV, c.seatXYZ_mm(3) + holder.stampHeight_mm];
        s.seat_UVW_mm = [seatUV, c.seatXYZ_mm(3)];
        s.rho_mm = hypot(s.top_UVW_mm(1), s.top_UVW_mm(2));
        s.phi_deg = atan2d(s.top_UVW_mm(2), s.top_UVW_mm(1));
        s.radialDir_deg = mod(yaw_deg + 90 - th + 180, 360) - 180;
        s.rimCorners_UVW_mm = [toWorld(rimH), repmat(c.rimZ_mm, 4, 1)];
        stamps(end+1) = s; %#ok<AGROW>
    end

    result.pose = pose;
    result.tags = [baseLoc, leftLoc, rightLoc, otherLocs];
    result.holder = hp;
    result.stamps = stamps;
    result.warnings = warnings;
end

function px = diagonalCrossing(cornerPx)
% Where the tag's diagonals cross in the image: the exact image of its centre.
    p = [cornerPx, ones(4, 1)];
    x = cross(cross(p(1,:), p(3,:)), cross(p(2,:), p(4,:)));
    px = x(1:2) / x(3);
end

function tag = findTag(tags, id, what)
    tag = tags([tags.id] == id);
    if isempty(tag)
        error("scene:tagNotSeen", "The %s, ID %d, was not detected in the photo.", what, id);
    end
    tag = tag(1);
end
