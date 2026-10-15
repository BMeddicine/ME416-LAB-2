function result = solveTagTarget(tags, layout, K, targetId, targetHeight_mm)
%SOLVETAGTARGET Base tag + one other tag -> that tag's centre in the base frame.
%   result = solveTagTarget(tags, layout, K, targetId, targetHeight_mm)
%     tags             struct array from detectTags
%     layout           from loadLayout (only tagSize_mm, tagCornerWinding,
%                      sideLengthTolerance and baseTag are used)
%     K                3x3 intrinsic matrix
%     targetId         ID of the tag to locate
%     targetHeight_mm  height of that tag above the plate (default 0)
%
%   A reduced solveScene for reach tests: only the base tag and the target
%   tag need to be in the photo (no ink pad, paper or holders). Fields:
%     pose        camera pose and table homography (estimateTablePose)
%     tags        the base and target tags located at their heights, with
%                 .role ("base" / "target"), for drawScene
%     baseCheck   base tag through the fitted pose vs its layout position
%     target      .tagId, .UVW_mm (tag centre), .rho_mm, .phi_deg, .yaw_deg,
%                 .centerPx (raw pixel), .meanSide_mm, .sideOk
%     warnings    string array of non-fatal problems

    if nargin < 5
        targetHeight_mm = 0;
    end
    warnings = strings(0, 1);
    tagSize_mm = requireValue(layout.tagSize_mm, "tagSize_mm");

    % ---- Pose from the base tag alone
    base = layout.baseTag;
    baseId = requireValue(base.id, "baseTag.id");
    baseCenter_mm = requireValue(base.centerUV_mm, "baseTag.centerUV_mm");
    baseYaw_deg = requireValue(base.yaw_deg, "baseTag.yaw_deg");
    baseHeight_mm = requireValue(base.height_mm, "baseTag.height_mm");
    if targetId == baseId
        error("tagTarget:sameTag", "The target tag can't be the base tag (ID %d).", baseId);
    end
    baseTag = findTag(tags, baseId, "base tag (baseTag.id)");
    targetTag = findTag(tags, targetId, "target tag");

    refUVW = tagCornersWorld(baseCenter_mm, baseYaw_deg, tagSize_mm, baseHeight_mm, ...
        layout.tagCornerWinding);
    pose = estimateTablePose(baseTag.cornerUndist_px, refUVW, K);
    pose.refTagIds = baseId;

    % ---- Locate both tags at their own heights
    baseLoc = locateTag(baseTag, pose, baseHeight_mm, tagSize_mm, layout.sideLengthTolerance);
    baseLoc.role = "base";
    targetLoc = locateTag(targetTag, pose, targetHeight_mm, tagSize_mm, layout.sideLengthTolerance);
    targetLoc.role = "target";
    if ~targetLoc.sideOk
        warnings(end+1) = sprintf(['Target tag %d: side %.1f mm vs %.1f mm expected (%+.1f%%). ' ...
            'Wrong target height, or a bad detection.'], targetId, targetLoc.meanSide_mm, ...
            tagSize_mm, 100 * targetLoc.sideError);
    end

    result.baseCheck.centerError_mm = norm(baseLoc.centerUV_mm - baseCenter_mm);
    result.baseCheck.yawError_deg = mod(baseLoc.yaw_deg - baseYaw_deg + 180, 360) - 180;

    t.tagId = targetId;
    t.UVW_mm = [targetLoc.centerUV_mm, targetHeight_mm];
    t.rho_mm = hypot(t.UVW_mm(1), t.UVW_mm(2));
    t.phi_deg = atan2d(t.UVW_mm(2), t.UVW_mm(1));
    t.yaw_deg = targetLoc.yaw_deg;
    t.centerPx = targetLoc.centerPx;
    t.meanSide_mm = targetLoc.meanSide_mm;
    t.sideOk = targetLoc.sideOk;

    result.pose = pose;
    result.tags = [baseLoc, targetLoc];
    result.target = t;
    result.warnings = warnings;
end

function tag = findTag(tags, id, what)
    tag = tags([tags.id] == id);
    if isempty(tag)
        error("scene:tagNotSeen", "The %s, ID %d, was not detected in the photo.", what, id);
    end
    tag = tag(1);
end
