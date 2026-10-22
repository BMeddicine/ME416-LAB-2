function layout = loadLayout(layoutPath)
%LOADLAYOUT Read the shared rack layout (config/layout.json).
%   layout = loadLayout(layoutPath) decodes the JSON file and normalizes it:
%     - JSON null becomes NaN (scalar) or NaN entries (arrays), so a value
%       that has not been measured yet is always isnan, never empty
%     - vectors are rows
%   Missing required values are checked where they are used (requireValue),
%   so the error names the exact field.

    if nargin < 1 || strlength(string(layoutPath)) == 0
        layoutPath = fullfile(repoRoot(), "config", "layout.json");
    end
    layout = jsondecode(fileread(layoutPath));
    layout = normalize(layout);

    layout.tagCornerWinding = lower(char(layout.tagCornerWinding));
    layout.layoutPath = string(layoutPath);
end

function value = normalize(value)
% Recursively turn [] (JSON null) into NaN and column vectors into rows.
    if isstruct(value)
        for k = 1:numel(value)
            names = fieldnames(value(k));
            for f = 1:numel(names)
                value(k).(names{f}) = normalize(value(k).(names{f}));
            end
        end
    elseif iscell(value)
        value = cellfun(@normalize, value, "UniformOutput", false);
    elseif isnumeric(value) && isempty(value)
        value = NaN;
    elseif isnumeric(value) && iscolumn(value)
        value = value.';
    end
end
