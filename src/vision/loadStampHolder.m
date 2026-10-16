function holder = loadStampHolder(holderPath)
%LOADSTAMPHOLDER Read the stamp holder geometry (config/stamp_holder.json).
%   holder = loadStampHolder() reads config/stamp_holder.json.
%   holder = loadStampHolder(holderPath) reads another file.
%   The geometry comes from the holder's OnShape STEP export; see the
%   file's _notes for the holder frame H and the stamp naming.

    if nargin < 1 || strlength(string(holderPath)) == 0
        holderPath = fullfile(repoRoot(), "config", "stamp_holder.json");
    end
    holder = jsondecode(fileread(holderPath));
    if iscell(holder.stamps)
        holder.stamps = [holder.stamps{:}];
    end
    for k = 1:numel(holder.stamps)
        holder.stamps(k).name = string(holder.stamps(k).name);
        holder.stamps(k).row = string(holder.stamps(k).row);
        holder.stamps(k).seatXYZ_mm = holder.stamps(k).seatXYZ_mm(:).';
    end
    holder.rimSize_mm = holder.rimSize_mm(:).';
    holder.holderPath = string(holderPath);
end
