function setup_paths()
%SETUP_PATHS  Add the project's source folders to the MATLAB path.
root = fileparts(mfilename('fullpath'));
addpath(genpath(fullfile(root, 'src')));
addpath(fullfile(root, 'tests'));
end
