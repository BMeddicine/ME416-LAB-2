function img = drawScene(photoImg, scene, radialDistortion)
%DRAWSCENE Annotated copy of the photo for checking the vision result by eye.
%   img = drawScene(photoImg, scene, radialDistortion) draws on the raw photo:
%     - every detected tag outline with its ID and role (magenta; red if
%       its side-length check failed)
%     - a 50 mm grid on the table plane W = 0 (yellow), with the U axis red
%       and the V axis green from the base axis. If the grid lines up with
%       the plate edges, the base frame is right.
%     - an optional scene.goal (.UVW_mm, .label) in red, projected from
%       its own height (locateTagTarget's target)
%   scene needs .pose and .tags. World points are projected with lens
%   distortion so they land on the raw photo pixels.

    pose = scene.pose;
    project = @(UVW) projectWorldToPixel(pose, UVW, radialDistortion);
    imgSize = max(size(photoImg, [1 2]));
    lineWidth_px = max(2, round(imgSize / 1500));
    fontSize_pt = min(72, max(14, round(imgSize / 120)));
    dotRadius_px = max(5, round(imgSize / 400));
    img = photoImg;

    % ---- Table grid, spanning every located tag plus a margin
    allUV_mm = [0 0; vertcat(scene.tags.centerUV_mm)];
    gridStep_mm = 50;
    lo = floor((min(allUV_mm, [], 1) - 100) / gridStep_mm) * gridStep_mm;
    hi = ceil((max(allUV_mm, [], 1) + 100) / gridStep_mm) * gridStep_mm;
    samples = 24;
    gridLines = zeros(0, 2 * samples);
    for U = lo(1):gridStep_mm:hi(1)
        V = linspace(lo(2), hi(2), samples).';
        gridLines(end+1, :) = reshape(project([repmat(U, samples, 1), V, zeros(samples, 1)]).', 1, []); %#ok<AGROW>
    end
    for V = lo(2):gridStep_mm:hi(2)
        U = linspace(lo(1), hi(1), samples).';
        gridLines(end+1, :) = reshape(project([U, repmat(V, samples, 1), zeros(samples, 1)]).', 1, []); %#ok<AGROW>
    end
    img = insertShape(img, "line", gridLines, "Color", "yellow", "LineWidth", max(1, lineWidth_px - 1), ...
        "Opacity", 0.6);

    axisLen_mm = 150;
    axisPx = project([0 0 0; axisLen_mm 0 0; 0 axisLen_mm 0]);
    img = insertShape(img, "line", [axisPx(1,:) axisPx(2,:); axisPx(1,:) axisPx(3,:)], ...
        "Color", {"red", "green"}, "LineWidth", 3 * lineWidth_px);
    img = insertText(img, [axisPx(2,:); axisPx(3,:)], ["U", "V"], "FontSize", fontSize_pt, ...
        "TextColor", "white", "BoxColor", {"red", "green"}, "AnchorPoint", "Center");

    % ---- Tags
    for t = scene.tags
        color = "magenta";
        if ~t.sideOk
            color = "red";
        end
        img = insertShape(img, "polygon", reshape(t.cornerPx.', 1, []), "Color", color, ...
            "LineWidth", lineWidth_px);
        img = insertShape(img, "filled-circle", [t.cornerPx(1,:) dotRadius_px], "Color", color);
        img = insertText(img, t.centerPx, sprintf("%d %s", t.id, t.role), "FontSize", fontSize_pt, ...
            "TextColor", "white", "BoxColor", color, "BoxOpacity", 0.6, "AnchorPoint", "Center");
    end

    % ---- Target
    if isfield(scene, "goal")
        img = drawTarget(img, project, scene.goal.UVW_mm, scene.goal.label, [255 40 40], dotRadius_px, fontSize_pt);
    end
end

function img = drawTarget(img, project, UVW_mm, label, color, dotRadius_px, fontSize_pt)
    px = project(UVW_mm);
    img = insertShape(img, "circle", [px 2 * dotRadius_px], "Color", color, "LineWidth", ...
        max(2, round(dotRadius_px / 2)));
    img = insertShape(img, "filled-circle", [px dotRadius_px / 2], "Color", color);
    img = insertText(img, px + [2.5 * dotRadius_px, -2.5 * dotRadius_px], label, ...
        "FontSize", fontSize_pt, "TextColor", "black", "BoxColor", color, "AnchorPoint", "LeftBottom");
end
