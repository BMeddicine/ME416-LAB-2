function result = locateTagTarget(targetId, photoPath, options)
%LOCATETAGTARGET Photo with the base tag + one other tag -> that tag in the base frame.
%   result = locateTagTarget(targetId) uses the newest photo in photos/.
%   result = locateTagTarget(targetId, photoPath) uses that photo.
%   result = locateTagTarget(targetId, photoPath, Name=Value) with options
%     TargetHeight_mm  height of the target tag above the plate (default 0)
%     LayoutPath       layout file (default config/layout.json)
%     Calibration      camera calibration .mat (default layout.calibrationFile),
%                      e.g. "calibration/webcamParams.mat" for the webcam
%     Save             write logs/tagtarget_<time>.json and an annotated .jpg (default true)
%     Show             open the annotated image in a figure (default false)
%
%   For reach tests: tools/move_to_tag.py reads the newest tagtarget_*.json
%   and drives the tool point to the tag centre. Only the base tag and the
%   target tag need to be in the photo. Same photo rules as locateScene.
%
%   See solveTagTarget for the fields of result.

    arguments
        targetId (1,1) double
        photoPath string = ""
        options.TargetHeight_mm (1,1) double = 0
        options.LayoutPath string = ""
        options.Calibration string = ""
        options.Save (1,1) logical = true
        options.Show (1,1) logical = false
    end

    root = repoRoot();
    layout = loadLayout(options.LayoutPath);

    % ---- Photo and calibration (same checks as locateScene)
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
        hint = "";
        if isequal(photoSize, fliplr(intrinsics.ImageSize))
            hint = " The photo was rotated (EXIF applied) somewhere during transfer; send the original file.";
        end
        error("locateScene:imageSize", ...
            "Photo is %dx%d (rows x cols) but the calibration is %dx%d. Use the same camera and resolution as the calibration.%s", ...
            photoSize, intrinsics.ImageSize, hint);
    end

    % ---- Detect and solve
    tags = detectTags(photoImg, layout.tagFamily, intrinsics);
    if isempty(tags)
        error("locateScene:noTags", "No %s tags found in %s.", layout.tagFamily, photoPath);
    end
    result = solveTagTarget(tags, layout, intrinsics.K, targetId, options.TargetHeight_mm);
    result.photoPath = string(photoPath);
    result.calibrationFile = string(calibrationFile);
    result.time = string(datetime("now", "Format", "yyyy-MM-dd HH:mm:ss"));
    result.detectedIds = [tags.id];

    printSummary(result);

    % ---- Outputs
    if options.Save || options.Show
        scene = struct("pose", result.pose, "tags", result.tags, "inkPad", [], "paper", [], ...
            "arm", [], "stamps", [], "goal", struct("UVW_mm", result.target.UVW_mm, ...
            "label", sprintf("target %d", targetId)));
        annotatedImg = drawScene(photoImg, scene, intrinsics.RadialDistortion);
    end
    if options.Save
        logDir = resolvePath(root, layout.logDir);
        if ~isfolder(logDir)
            mkdir(logDir);
        end
        stamp = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
        jsonPath = fullfile(logDir, "tagtarget_" + stamp + ".json");
        imagePath = fullfile(logDir, "tagtarget_" + stamp + ".jpg");
        out = result;
        out.tags = rmfield(result.tags, ["cornerPx", "cornerUV_mm"]);
        writelines(jsonencode(out, PrettyPrint=true), jsonPath);
        imwrite(annotatedImg, imagePath, Quality=90);
        fprintf("Saved %s\n      %s\n", jsonPath, imagePath);
    end
    if options.Show
        figure("Name", "locateTagTarget", "Color", "w");
        imshow(annotatedImg);
    end
end

% -------------------------------------------------------------------------
function intrinsics = loadIntrinsics(calibrationPath)
% The first cameraParameters object in the file, whatever the variable is called
% (cameraParams for the iPhone, cameraParamsWebcam for the webcam).
    saved = load(calibrationPath);
    names = fieldnames(saved);
    for k = 1:numel(names)
        if isa(saved.(names{k}), "cameraParameters")
            intrinsics = saved.(names{k}).Intrinsics;
            return
        end
    end
    error("locateTagTarget:noCalibration", "No cameraParameters object in %s.", calibrationPath);
end

function path = resolvePath(root, path)
% Layout paths are relative to the repository root unless absolute.
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
        error("locateScene:noPhoto", "No .jpg/.jpeg/.png photo in %s.", photoDir);
    end
    [~, newest] = max([files.datenum]);
    photoPath = string(fullfile(files(newest).folder, files(newest).name));
end

function printSummary(result)
    p = result.pose;
    t = result.target;
    fprintf("\n== Tag target from %s ==\n", result.photoPath);
    fprintf("Tags detected: %s\n", mat2str(result.detectedIds));
    fprintf("Base tag %d: reprojection RMS %.2f px (%.2f mm on the plane)\n", ...
        p.refTagIds, p.reprojRms_px, p.reprojRms_mm);
    fprintf("Camera at [%.0f, %.0f, %.0f] mm\n", p.cameraCenter_mm);
    fprintf("Target tag %d: [%7.1f, %7.1f, %5.1f] mm  rho %6.1f  phi %6.1f  yaw %6.1f  side %.1f mm\n", ...
        t.tagId, t.UVW_mm, t.rho_mm, t.phi_deg, t.yaw_deg, t.meanSide_mm);
    for w = result.warnings.'
        fprintf("WARNING: %s\n", w);
    end
    fprintf("\n");
end
