function scene = solveScene(tags, layout, K, requiredLetters)
%SOLVESCENE Detected tags + rack layout -> every target in the base frame.
%   scene = solveScene(tags, layout, K, requiredLetters)
%     tags             struct array from detectTags (needs id, cornerUV_px,
%                      cornerUndist_px, centerUV_px)
%     layout           from loadLayout
%     K                3x3 intrinsic matrix
%     requiredLetters  letters the run will stamp (char or string), checked
%                      to have a stamp target; '' to skip the check
%
%   This is the photo-independent part of locateScene, so it can be tested
%   with synthetic corners. Output fields:
%     pose        camera pose and table homography (estimateTablePose)
%     tags        every detected tag located at its configured height,
%                 with .role ("base", "inkPad", "paper", "arm", "holder A",
%                 or "unknown")
%     baseCheck   base tag located through the fitted pose vs its layout
%                 position (should be ~0; large means a bad fit)
%     inkPad      .UVW_mm, .rho_mm, .phi_deg, .yaw_deg, .tagId
%     paper       same fields
%     arm         same fields, or [] if not configured / not seen
%     stamps      struct array, one per configured holder: .letter, .tagId,
%                 .disc_UVW_mm, .rho_mm, .phi_deg, .yaw_deg, .source,
%                 .detected, .onshapeResidual_mm, .onshapeYawResidual_deg
%     warnings    string array of non-fatal problems

    if nargin < 4
        requiredLetters = '';
    end
    warnings = strings(0, 1);
    tagSize_mm = requireValue(layout.tagSize_mm, "tagSize_mm");
    winding = layout.tagCornerWinding;
    tagIds = [tags.id];

    % ---- Reference points: base tag (always) + OnShape holder tags (optional)
    base = layout.baseTag;
    baseId = requireValue(base.id, "baseTag.id");
    baseCenter_mm = requireValue(base.centerUV_mm, "baseTag.centerUV_mm");
    baseYaw_deg = requireValue(base.yaw_deg, "baseTag.yaw_deg");
    baseHeight_mm = requireValue(base.height_mm, "baseTag.height_mm");
    baseTag = findTag(tags, baseId, "base tag (baseTag.id)");

    refPx = baseTag.cornerUndist_px;
    refUVW = tagCornersWorld(baseCenter_mm, baseYaw_deg, tagSize_mm, baseHeight_mm, winding);
    refIds = baseId;

    holders = layout.stampHolders;
    if layout.useHolderTagsAsReference
        for k = 1:numel(holders)
            h = holders(k);
            if isnan(h.tagId) || any(isnan(h.onshapeCenterUV_mm)) || isnan(h.onshapeYaw_deg) ...
                    || ~any(tagIds == h.tagId)
                continue
            end
            if abs(h.tagHeight_mm - baseHeight_mm) > 1e-9
                warnings(end+1) = sprintf(['Holder %s tag not used as a reference: it is at ' ...
                    '%.1f mm, the base tag at %.1f mm (one homography needs one plane).'], ...
                    h.letter, h.tagHeight_mm, baseHeight_mm); %#ok<AGROW>
                continue
            end
            refPx = [refPx; tags(tagIds == h.tagId).cornerUndist_px]; %#ok<AGROW>
            refUVW = [refUVW; tagCornersWorld(h.onshapeCenterUV_mm, h.onshapeYaw_deg, ...
                tagSize_mm, h.tagHeight_mm, winding)]; %#ok<AGROW>
            refIds(end+1) = h.tagId; %#ok<AGROW>
        end
    end

    pose = estimateTablePose(refPx, refUVW, K);
    pose.refTagIds = refIds;

    % ---- Locate every detected tag at its configured height
    located = [];
    for k = 1:numel(tags)
        [role, height_mm] = tagRole(tags(k).id, layout);
        if isnan(height_mm)
            warnings(end+1) = sprintf("Tag %d (%s) has no height in layout.json; located at W = 0.", ...
                tags(k).id, role); %#ok<AGROW>
            height_mm = 0;
        end
        loc = locateTag(tags(k), pose, height_mm, tagSize_mm, layout.sideLengthTolerance);
        loc.role = role;
        if ~loc.sideOk
            warnings(end+1) = sprintf(['Tag %d (%s): side %.1f mm vs %.1f mm expected (%+.1f%%). ' ...
                'Wrong height_mm for this tag, or a bad detection.'], tags(k).id, role, ...
                loc.meanSide_mm, tagSize_mm, 100 * loc.sideError); %#ok<AGROW>
        end
        located = [located, loc]; %#ok<AGROW>
    end
    locatedIds = [located.id];

    baseLoc = located(locatedIds == baseId);
    scene.baseCheck.centerError_mm = norm(baseLoc.centerUV_mm - baseCenter_mm);
    scene.baseCheck.yawError_deg = wrapAngle(baseLoc.yaw_deg - baseYaw_deg);

    % ---- Ink pad and paper (required, found at run time)
    scene.inkPad = locateTarget(located, layout.inkPad, "inkPad", true);
    scene.paper = locateTarget(located, layout.paper, "paper", true);

    % ---- Arm tag (optional cross-check)
    scene.arm = [];
    if ~isnan(layout.arm.id)
        if any(locatedIds == layout.arm.id)
            scene.arm = locateTarget(located, layout.arm, "arm", false);
        else
            warnings(end+1) = sprintf("Arm tag %d not seen.", layout.arm.id); %#ok<AGROW>
        end
    end

    % ---- Stamps
    scene.stamps = struct('letter', {}, 'tagId', {}, 'disc_UVW_mm', {}, 'rho_mm', {}, ...
        'phi_deg', {}, 'yaw_deg', {}, 'source', {}, 'detected', {}, ...
        'onshapeResidual_mm', {}, 'onshapeYawResidual_deg', {});
    if ~any(strcmp(layout.holderSource, ["tags", "onshape"]))
        error("layout:holderSource", "holderSource must be ""tags"" or ""onshape"", got ""%s"".", ...
            layout.holderSource);
    end
    for k = 1:numel(holders)
        h = holders(k);
        if isnan(h.tagId)
            continue
        end
        tagToDisc_mm = requireValue(layout.stamp.tagToDisc_mm, "stamp.tagToDisc_mm");
        discHeight_mm = requireValue(layout.stamp.discHeight_mm, "stamp.discHeight_mm");
        haveOnshape = ~any(isnan(h.onshapeCenterUV_mm)) && ~isnan(h.onshapeYaw_deg);
        detected = any(locatedIds == h.tagId);

        s = struct('letter', h.letter, 'tagId', h.tagId, 'disc_UVW_mm', [], 'rho_mm', NaN, ...
            'phi_deg', NaN, 'yaw_deg', NaN, 'source', "", 'detected', detected, ...
            'onshapeResidual_mm', NaN, 'onshapeYawResidual_deg', NaN);
        if detected
            loc = located(locatedIds == h.tagId);
            if haveOnshape
                s.onshapeResidual_mm = norm(loc.centerUV_mm - h.onshapeCenterUV_mm);
                s.onshapeYawResidual_deg = wrapAngle(loc.yaw_deg - h.onshapeYaw_deg);
            end
        end

        if strcmp(layout.holderSource, "onshape")
            if ~haveOnshape
                error("layout:missingValue", ['holderSource is "onshape" but holder %s has no ' ...
                    'onshapeCenterUV_mm / onshapeYaw_deg.'], h.letter);
            end
            tagCenter_mm = h.onshapeCenterUV_mm;
            tagYaw_deg = h.onshapeYaw_deg;
            s.source = "onshape";
        elseif detected
            tagCenter_mm = loc.centerUV_mm;
            tagYaw_deg = loc.yaw_deg;
            s.source = "tag";
        else
            warnings(end+1) = sprintf("Holder %s (tag %d) not seen: no target for it.", ...
                h.letter, h.tagId); %#ok<AGROW>
            continue
        end

        discUV_mm = applyTagOffset(tagCenter_mm, tagYaw_deg, tagToDisc_mm);
        s.disc_UVW_mm = [discUV_mm, discHeight_mm];
        [s.rho_mm, s.phi_deg] = polarOf(discUV_mm);
        s.yaw_deg = tagYaw_deg;
        scene.stamps(end+1) = s;
    end

    available = upper(char([scene.stamps.letter]));
    missing = setdiff(unique(upper(char(requiredLetters))), [available ' ']);
    if ~isempty(missing)
        error("scene:missingStamps", "No stamp target for letter(s) %s. Check the holder tags/IDs.", ...
            strjoin(cellstr(missing.'), ", "));
    end

    scene.pose = pose;
    scene.tags = located;
    scene.warnings = warnings;
end

% -------------------------------------------------------------------------
function tag = findTag(tags, id, what)
    tag = tags([tags.id] == id);
    if isempty(tag)
        error("scene:tagNotSeen", "The %s, ID %d, was not detected in the photo.", what, id);
    end
    tag = tag(1);
end

function [role, height_mm] = tagRole(id, layout)
% Role name and configured tag height for a detected ID (NaN height if unknown).
    role = "unknown";
    height_mm = 0;
    if id == layout.baseTag.id
        role = "base";  height_mm = layout.baseTag.height_mm;
    elseif id == layout.inkPad.id
        role = "inkPad";  height_mm = layout.inkPad.height_mm;
    elseif id == layout.paper.id
        role = "paper";  height_mm = layout.paper.height_mm;
    elseif id == layout.arm.id
        role = "arm";  height_mm = layout.arm.height_mm;
    else
        for k = 1:numel(layout.stampHolders)
            if id == layout.stampHolders(k).tagId
                role = "holder " + layout.stampHolders(k).letter;
                height_mm = layout.stampHolders(k).tagHeight_mm;
                return
            end
        end
    end
end

function target = locateTarget(located, cfg, name, required)
% Tag-based target: tag position + offset in the tag frame, at its target height.
    id = requireValue(cfg.id, name + ".id");
    loc = located([located.id] == id);
    if isempty(loc)
        if required
            error("scene:tagNotSeen", "The %s tag, ID %d, was not detected in the photo.", name, id);
        end
        target = [];
        return
    end
    offset_mm = requireValue(cfg.offsetTag_mm, name + ".offsetTag_mm");
    targetHeight_mm = loc.height_mm;
    if isfield(cfg, "targetHeight_mm") && ~isnan(cfg.targetHeight_mm)
        targetHeight_mm = cfg.targetHeight_mm;
    end
    UV_mm = applyTagOffset(loc.centerUV_mm, loc.yaw_deg, offset_mm);
    target.tagId = id;
    target.UVW_mm = [UV_mm, targetHeight_mm];
    [target.rho_mm, target.phi_deg] = polarOf(UV_mm);
    target.yaw_deg = loc.yaw_deg;
end

function [rho_mm, phi_deg] = polarOf(UV_mm)
    rho_mm = hypot(UV_mm(1), UV_mm(2));
    phi_deg = atan2d(UV_mm(2), UV_mm(1));
end

function a = wrapAngle(a)
    a = mod(a + 180, 360) - 180;
end
