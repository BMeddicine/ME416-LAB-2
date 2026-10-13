function scene = locateScene(photoPath, options)
%LOCATESCENE Photo of the rack -> stamp, ink pad and paper targets in the base frame.
%   scene = locateScene() uses the newest photo in photos/ and config/layout.json.
%   scene = locateScene(photoPath) uses that photo.
%   scene = locateScene(photoPath, Name=Value) with options
%     LayoutPath       layout file (default config/layout.json)
%     RequiredLetters  letters the run needs, e.g. "BRYSON" (default: none)
%     Save             write logs/scene_<time>.json and an annotated .jpg (default true)
%     Show             open the annotated image in a figure (default false)
%
%   Photo rules: same phone, same 0.5x lens and same resolution as the
%   calibration photos, JPEG ("Most Compatible" on the iPhone). imread
%   ignores the EXIF rotation, exactly as during calibration, so the image
%   size must match cameraParams.ImageSize.
%
%   See solveScene for the fields of scene.

    arguments
        photoPath string = ""
        options.LayoutPath string = ""
        options.RequiredLetters string = ""
        options.Save (1,1) logical = true
        options.Show (1,1) logical = false
    end

    root = repoRoot();
    layout = loadLayout(options.LayoutPath);

    % ---- Photo
    if photoPath == ""
        photoPath = newestPhoto(resolvePath(root, layout.photoDir));
    end
    photoImg = imread(photoPath);

    % ---- Calibration
    calib = load(resolvePath(root, layout.calibrationFile), "cameraParams");
    intrinsics = calib.cameraParams.Intrinsics;
    photoSize = [size(photoImg, 1), size(photoImg, 2)];
    if ~isequal(photoSize, intrinsics.ImageSize)
        hint = "";
        if isequal(photoSize, fliplr(intrinsics.ImageSize))
            hint = " The photo was rotated (EXIF applied) somewhere during transfer; send the original file.";
        end
        error("locateScene:imageSize", ...
            "Photo is %dx%d (rows x cols) but the calibration is %dx%d. Use the 0.5x lens at the calibration resolution.%s", ...
            photoSize, intrinsics.ImageSize, hint);
    end

    % ---- Detect and solve
    tags = detectTags(photoImg, layout.tagFamily, intrinsics);
    if isempty(tags)
        error("locateScene:noTags", "No %s tags found in %s.", layout.tagFamily, photoPath);
    end
    scene = solveScene(tags, layout, intrinsics.K, options.RequiredLetters);
    scene.photoPath = string(photoPath);
    scene.time = string(datetime("now", "Format", "yyyy-MM-dd HH:mm:ss"));
    scene.calibrationReprojError_px = calib.cameraParams.MeanReprojectionError;

    printSummary(scene);

    % ---- Outputs
    if options.Save || options.Show
        annotatedImg = drawScene(photoImg, scene, intrinsics.RadialDistortion);
    end
    if options.Save
        logDir = resolvePath(root, layout.logDir);
        if ~isfolder(logDir)
            mkdir(logDir);
        end
        stamp = string(datetime("now", "Format", "yyyyMMdd_HHmmss"));
        jsonPath = fullfile(logDir, "scene_" + stamp + ".json");
        imagePath = fullfile(logDir, "scene_" + stamp + ".jpg");
        writelines(jsonencode(sceneForJson(scene), PrettyPrint=true), jsonPath);
        imwrite(annotatedImg, imagePath, Quality=90);
        fprintf("Saved %s\n      %s\n", jsonPath, imagePath);
    end
    if options.Show
        figure("Name", "locateScene", "Color", "w");
        imshow(annotatedImg);
    end
end

% -------------------------------------------------------------------------
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

function printSummary(scene)
    p = scene.pose;
    fprintf("\n== Scene from %s ==\n", scene.photoPath);
    fprintf("Reference tags %s: %d corners, reprojection RMS %.2f px (%.2f mm on the plane)\n", ...
        mat2str(p.refTagIds), p.nRefPoints, p.reprojRms_px, p.reprojRms_mm);
    fprintf("Camera at [%.0f, %.0f, %.0f] mm\n", p.cameraCenter_mm);
    fprintf("Base tag check: %.2f mm, %.2f deg from its layout position\n", ...
        scene.baseCheck.centerError_mm, scene.baseCheck.yawError_deg);
    printTarget("Ink pad", scene.inkPad);
    printTarget("Paper", scene.paper);
    if ~isempty(scene.arm)
        printTarget("Arm tag", scene.arm);
    end
    for s = scene.stamps
        extra = "";
        if ~isnan(s.onshapeResidual_mm)
            extra = sprintf("  (tag vs OnShape %.1f mm, %.1f deg)", ...
                s.onshapeResidual_mm, s.onshapeYawResidual_deg);
        end
        fprintf("Stamp %s: disc [%7.1f, %7.1f, %5.1f] mm  rho %6.1f  phi %6.1f  [%s]%s\n", ...
            s.letter, s.disc_UVW_mm, s.rho_mm, s.phi_deg, s.source, extra);
    end
    for w = scene.warnings.'
        fprintf("WARNING: %s\n", w);
    end
    fprintf("\n");
end

function printTarget(name, t)
    fprintf("%-8s [%7.1f, %7.1f, %5.1f] mm  rho %6.1f  phi %6.1f  yaw %6.1f\n", ...
        name + ":", t.UVW_mm, t.rho_mm, t.phi_deg, t.yaw_deg);
end

function out = sceneForJson(scene)
% Drop the bulky matrices that are not useful in the log.
    out = scene;
    out.tags = rmfield(scene.tags, ["cornerPx", "cornerUV_mm"]);
end
