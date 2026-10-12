function root = repoRoot()
%REPOROOT Absolute path of the repository (two levels above this file).
    root = string(fileparts(fileparts(fileparts(mfilename("fullpath")))));
end
