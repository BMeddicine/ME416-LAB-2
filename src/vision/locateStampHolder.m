function result = locateStampHolder(photoPath, options)
%LOCATESTAMPHOLDER Photo with the base tag + the holder's tags 1 and 3 -> all 26 stamp tops.
%   result = locateStampHolder() uses the newest photo in photos/.
%   result = locateStampHolder(photoPath) uses that photo.
%   result = locateStampHolder(photoPath, Name=Value) with options
%     LayoutPath   layout file (default config/layout.json)
%     HolderPath   holder geometry (default config/stamp_holder.json)
%     Calibration  camera calibration .mat (default layout.calibrationFile),
%                  e.g. "calibration/webcamParams.mat" for the webcam
%     TagHints     JSON of OpenCV detections [{id, cornerPx}] (written by the
%                  Python webcam tools); tags readAprilTag misses are read
%                  again inside these outlines (see detectTags)
%     Save         write logs/stampholder_<time>.json and an annotated .jpg (default true)
%     Show         open the annotated image in a figure (default false)
%
%   The annotated image is the visual check. For each of the 26 positions it draws
%     - the well opening (outline at rim height): should sit on the real well
%     - the stamp top (dot at seat + 30 mm): should sit on the top of a loaded stamp
%     - a thin line from the seat up to the top
%   front/lower row in cyan, back/upper row in green, plus the holder tags
%   and the base-frame grid from drawScene.
%
%   See solveStampHolder for the fields of result.
%   From Python with the webcam: tools/check_stamp_holder.py.

    arguments
        photoPath string = ""
        options.LayoutPath string = ""
        options.HolderPath string = ""
        options.Calibration string = ""
        options.TagHints string = ""
        options.Save (1,1) logical = true
        options.Show (1,1) logical = false
    end

    root = repoRoot();
    layout = loadLayout(options.LayoutPath);
    holder = loadStampHolder(options.HolderPath);

    % ---- Photo and calibration (same checks as locateTagTarget)
    if photoPath == ""
        photoPath = newestPhoto(resolvePath(root, layout.photoDir));
    end
    photoImg = imread(photoPath);
    calibrationFile = options.Calibration;
    if calibrationFile == ""
        calibrationFile = layout.calibrationFile;
    end
    intrinsics = loadIntrinsics(resolvePath(root, calibrationFile));
    photoSize = [size(photoImg, 1), size(photoImg, 2)];
    if ~isequal(photoSize, intrinsics.ImageSize)
        error("vision:imageSize", ...
            "Photo is %dx%d (rows x cols) but the calibration is %dx%d. Use the same camera and resolution as the calibration.", ...
            photoSize, intrinsics.ImageSize);
    end

    % ---- Detect and solve
    hints = struct("id", {}, "cornerPx", {});
    if options.TagHints ~= ""
        hints = jsondecode(fileread(options.TagHints));
    end
    tags = detectTags(photoImg, layout.tagFamily, intrinsics, hints);
    if isempty(tags)
        error("vision:noTags", "No %s tags found in %s.", layout.tagFamily, photoPath);
    end
    result = solveStampHolder(tags, layout, intrinsics.K, holder);
    result.photoPath = string(photoPath);
    result.calibrationFile = string(calibrationFile);
    result.time = string(datetime("now", "Format", "yyyy-MM-dd HH:mm:ss"));
    result.detectedIds = [tags.id];

    printSummary(result);

    % ---- Outputs
    if options.Save || options.Show
        scene = struct("pose", result.pose, "tags", result.tags);
        annotatedImg = drawScene(photoImg, scene, intrinsics.RadialDistortion);
        annotatedImg = drawHolder(annotatedImg, result, intrinsics.RadialDistortion);
    end
    if options.Save
        logDir = resolvePath(root, layout.logDir);
        if ~isfolder(logDir)
            mkdir(logDir);
        end
        stamp = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
        jsonPath = fullfile(logDir, "stampholder_" + stamp + ".json");
        imagePath = fullfile(logDir, "stampholder_" + stamp + ".jpg");
        out = result;
        out.tags = rmfield(result.tags, ["cornerPx", "cornerUV_mm"]);
        out.stamps = rmfield(result.stamps, "rimCorners_UVW_mm");
        % Pixel of each stamp top on the raw photo, for clicking on it (tools/stamp_hover_gui.py)
        topPx = projectWorldToPixel(result.pose, vertcat(result.stamps.top_UVW_mm), ...
            intrinsics.RadialDistortion);
        for k = 1:numel(out.stamps)
            out.stamps(k).topPx = topPx(k,:);
        end
        % Lens model, so Python can project world points (ink pad, paper) onto the raw photo
        out.radialDistortion = intrinsics.RadialDistortion;
        writelines(jsonencode(out, PrettyPrint=true), jsonPath);
        imwrite(annotatedImg, imagePath, Quality=95);
        fprintf("Saved %s\n      %s\n", jsonPath, imagePath);
    end
    if options.Show
        figure("Name", "locateStampHolder", "Color", "w");
        imshow(annotatedImg);
    end
end

% -------------------------------------------------------------------------
function img = drawHolder(img, result, radialDistortion)
    project = @(UVW) projectWorldToPixel(result.pose, UVW, radialDistortion);
    imgSize = max(size(img, [1 2]));
    lineWidth_px = max(1, round(imgSize / 1500));
    dotRadius_px = max(2, round(imgSize / 450));
    fontSize_pt = max(8, round(imgSize / 220));
    colors = struct("lower", [0 220 255], "upper", [60 255 60]);

    % Holder axes at tag height: left tag -> right tag (x), and 40 mm of +y
    hp = result.holder;
    h = result.tags(2).height_mm;
    yHat = [-sind(hp.yaw_deg), cosd(hp.yaw_deg)];
    yTip_UV_mm = hp.origin_UV_mm + 40 * yHat;
    axisPx = project([hp.leftTag_UV_mm, h; hp.rightTag_UV_mm, h; hp.origin_UV_mm, h; yTip_UV_mm, h]);
    img = insertShape(img, "line", [axisPx(1,:) axisPx(2,:); axisPx(3,:) axisPx(4,:)], ...
        "Color", {"red", "green"}, "LineWidth", 2 * lineWidth_px);

    for s = result.stamps
        color = colors.(s.row);
        rimPx = project(s.rimCorners_UVW_mm);
        img = insertShape(img, "polygon", reshape(rimPx.', 1, []), "Color", color, "LineWidth", lineWidth_px);
        ends = project([s.seat_UVW_mm; s.top_UVW_mm]);
        img = insertShape(img, "line", reshape(ends.', 1, []), "Color", color, "LineWidth", 1);
        img = insertShape(img, "filled-circle", [ends(2,:) dotRadius_px], "Color", color);
        img = insertText(img, ends(2,:) + [dotRadius_px, -dotRadius_px], s.name, "FontSize", fontSize_pt, ...
            "TextColor", "black", "BoxColor", color, "BoxOpacity", 0.6, "AnchorPoint", "LeftBottom");
    end
end

function printSummary(result)
    p = result.pose;
    hp = result.holder;
    fprintf("\n== Stamp holder from %s ==\n", result.photoPath);
    fprintf("Tags detected: %s\n", mat2str(result.detectedIds));
    fprintf("Base tag %d: reprojection RMS %.2f px (%.2f mm on the plane)\n", ...
        p.refTagIds, p.reprojRms_px, p.reprojRms_mm);
    fprintf("Camera at [%.0f, %.0f, %.0f] mm\n", p.cameraCenter_mm);
    fprintf("Holder tags: L [%7.1f, %7.1f]  R [%7.1f, %7.1f] mm, %.1f mm apart (CAD %+.1f mm)\n", ...
        hp.leftTag_UV_mm, hp.rightTag_UV_mm, hp.baseline_mm, hp.baselineError_mm);
    fprintf("Holder origin [%7.1f, %7.1f] mm, x axis at %.1f deg\n", hp.origin_UV_mm, hp.yaw_deg);
    fprintf("%-4s %8s %8s %6s %7s %7s\n", "", "U", "V", "W", "rho", "phi");
    for s = result.stamps
        fprintf("%-4s %8.1f %8.1f %6.1f %7.1f %7.1f\n", s.name, s.top_UVW_mm, s.rho_mm, s.phi_deg);
    end
    for w = result.warnings.'
        fprintf("WARNING: %s\n", w);
    end
    fprintf("\n");
end

function intrinsics = loadIntrinsics(calibrationPath)
% The first cameraParameters object in the file, whatever the variable is called.
    saved = load(calibrationPath);
    names = fieldnames(saved);
    for k = 1:numel(names)
        if isa(saved.(names{k}), "cameraParameters")
            intrinsics = saved.(names{k}).Intrinsics;
            return
        end
    end
    error("locateStampHolder:noCalibration", "No cameraParameters object in %s.", calibrationPath);
end

function path = resolvePath(root, path)
    path = string(path);
    if ~startsWith(path, ["/", "\"]) && ~contains(path, ":")
        path = fullfile(root, path);
    end
end

function photoPath = newestPhoto(photoDir)
    files = [dir(fullfile(photoDir, "*.jpg")); dir(fullfile(photoDir, "*.jpeg")); ...
             dir(fullfile(photoDir, "*.png"))];
    files = files(~startsWith({files.name}, "."));
    if isempty(files)
        error("vision:noPhoto", "No .jpg/.jpeg/.png photo in %s.", photoDir);
    end
    [~, newest] = max([files.datenum]);
    photoPath = string(fullfile(files(newest).folder, files(newest).name));
end
