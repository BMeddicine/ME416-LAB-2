function tests = testStampHolder
%TESTSTAMPHOLDER Synthetic test for solveStampHolder (src/vision).
%   The holder tags are rendered on their tilted pads (corners at different
%   heights), so this also checks that the tilt does not bias the centres.
%   Run:  addpath(genpath('src')); runtests('tests')
    tests = functiontests(localfunctions);
end

function setupOnce(testCase)
    root = fileparts(fileparts(mfilename("fullpath")));
    addpath(genpath(fullfile(root, "src")));
    layout = loadLayout(fullfile(root, "config", "layout.json"));
    layout.tagSize_mm = 50;
    layout.tagCornerWinding = 'ccw';
    layout.baseTag = struct('id', 0, 'centerUV_mm', [57.5 -84.5], 'yaw_deg', 0, 'height_mm', 0);
    testCase.TestData.layout = layout;
    testCase.TestData.holder = loadStampHolder(fullfile(root, "config", "stamp_holder.json"));
    testCase.TestData.pose = webcamLikeCamera();
end

function testRecoversEveryStampTop(testCase)
    layout = testCase.TestData.layout;
    holder = testCase.TestData.holder;
    pose = testCase.TestData.pose;
    origin = [40 210];
    yaw = 12;
    tags = renderAll(layout, holder, pose, origin, yaw);
    result = solveStampHolder(tags, layout, pose.K, holder);

    tol = 1e-6;
    verifyLessThan(testCase, result.baseCheck.centerError_mm, tol);
    verifyEqual(testCase, result.holder.origin_UV_mm, origin, AbsTol=tol);
    verifyEqual(testCase, result.holder.yaw_deg, yaw, AbsTol=tol);
    verifyEqual(testCase, result.holder.baselineError_mm, 0, AbsTol=1e-3);
    verifyNumElements(testCase, result.stamps, 26);
    for k = 1:numel(holder.stamps)
        c = holder.stamps(k);
        expected = [toWorld(c.seatXYZ_mm(1:2), origin, yaw), c.seatXYZ_mm(3) + holder.stampHeight_mm];
        verifyEqual(testCase, result.stamps(k).top_UVW_mm, expected, AbsTol=tol);
        verifyEqual(testCase, result.stamps(k).name, c.name);
    end
    verifyEmpty(testCase, result.warnings);
end

function testWrongTagIdsAreFlagged(testCase)
    % Swapping left and right flips the holder by 180 deg; the baseline still
    % matches, so the check is that x points the wrong way.
    layout = testCase.TestData.layout;
    holder = testCase.TestData.holder;
    pose = testCase.TestData.pose;
    tags = renderAll(layout, holder, pose, [40 210], 12);
    swapped = holder;
    swapped.leftTagId = holder.rightTagId;
    swapped.rightTagId = holder.leftTagId;
    result = solveStampHolder(tags, layout, pose.K, swapped);
    verifyEqual(testCase, result.holder.yaw_deg, 12 - 180, AbsTol=1e-6);
end

function testMissingHolderTag(testCase)
    layout = testCase.TestData.layout;
    holder = testCase.TestData.holder;
    pose = testCase.TestData.pose;
    tags = renderAll(layout, holder, pose, [40 210], 12);
    tags = tags([tags.id] ~= holder.rightTagId);
    verifyError(testCase, @() solveStampHolder(tags, layout, pose.K, holder), "scene:tagNotSeen");
end

% -------------------------------------------------------------------------
function uv = toWorld(xy, origin, yaw)
    uv = origin + xy(1) * [cosd(yaw) sind(yaw)] + xy(2) * [-sind(yaw) cosd(yaw)];
end

function tags = renderAll(layout, holder, pose, origin, yaw)
    tags = struct('id', {}, 'cornerUV_px', {}, 'cornerUndist_px', {}, 'centerUV_px', {});
    base = layout.baseTag;
    tags = addTag(tags, base.id, tagCornersWorld(base.centerUV_mm, base.yaw_deg, layout.tagSize_mm, ...
        base.height_mm, layout.tagCornerWinding), pose);
    sides = ["left", "right"];
    ids = [holder.leftTagId, holder.rightTagId];
    halfBaseline = holder.tagBaseline_mm / 2;
    for k = 1:2
        n = holder.tagNormals.(sides(k))(:).';
        n = n / norm(n);
        e1 = [-n(2), n(1), 0] / hypot(n(1), n(2));      % horizontal edge of the pad
        e2 = cross(n, e1);
        centerH = [(2 * k - 3) * halfBaseline, 0, holder.tagCenterHeight_mm];
        a = layout.tagSize_mm / 2;
        cornersH = centerH + [-a -a; a -a; a a; -a a] * [e1; e2];
        cornersW = [toWorld(cornersH(1,1:2), origin, yaw); toWorld(cornersH(2,1:2), origin, yaw); ...
                    toWorld(cornersH(3,1:2), origin, yaw); toWorld(cornersH(4,1:2), origin, yaw)];
        tags = addTag(tags, ids(k), [cornersW, cornersH(:,3)], pose);
    end
end

function tags = addTag(tags, id, cornersUVW_mm, pose)
    px = projectWorldToPixel(pose, cornersUVW_mm);
    tags(end+1) = struct('id', id, 'cornerUV_px', px, 'cornerUndist_px', px, 'centerUV_px', mean(px, 1));
end

function pose = webcamLikeCamera()
% 640x480-like camera about 550 mm above the plate, tilted ~10 deg.
    K = [600 0 320; 0 600 240; 0 0 1];
    lookDown = [1 0 0; 0 -1 0; 0 0 -1];
    tilt = [1 0 0; 0 cosd(10) -sind(10); 0 sind(10) cosd(10)] * ...
           [cosd(4) 0 sind(4); 0 1 0; -sind(4) 0 cosd(4)];
    R = tilt * lookDown;
    cameraCenter_mm = [60 40 550];
    pose.K = K;
    pose.R = R;
    pose.t = -R * cameraCenter_mm.';
    pose.cameraCenter_mm = cameraCenter_mm;
end
