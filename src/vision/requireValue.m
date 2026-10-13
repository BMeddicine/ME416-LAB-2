function value = requireValue(value, fieldName)
%REQUIREVALUE Stop with a clear message if a layout value is not filled in.
%   value = requireValue(value, "baseTag.yaw_deg") returns value unchanged,
%   or errors if it is empty or contains NaN (JSON null in layout.json).

    if isempty(value) || (isnumeric(value) && any(isnan(value(:))))
        error("layout:missingValue", ...
            "config/layout.json: '%s' is not filled in (null). Measure it or take it from OnShape.", ...
            fieldName);
    end
end
