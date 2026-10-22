function root = repoRoot()
%REPOROOT Absolute path of the repository (one level above this file).
    root = string(fileparts(fileparts(mfilename("fullpath"))));
end
