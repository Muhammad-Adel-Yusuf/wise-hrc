% Regenerates the compact Fig. 3 of the T-RO manuscript (fig_combined.pdf, 7.16 x 6.0 in, 8-pt text)
% from the four recorded sessions. Run from any folder in MATLAB.
here = fileparts(mfilename('fullpath'));
addpath(fullfile(here, '..', '..'));   % POMDP model folder (helper functions)
addpath(here);                         % plotScenariosHRC_compact.m
r = fullfile(here, '..');              % results folder with the scenario recordings
files = {fullfile(r,'scenario_P01_adaptive_collaborative_20261009_035710','scenario_P01_adaptive_collaborative_20261009_035710.mat'), ...
         fullfile(r,'scenario_P01_adaptive_assistive_20261009_040743','scenario_P01_adaptive_assistive_20261009_040743.mat'), ...
         fullfile(r,'scenario_P01_adaptive_solo_20261009_040950','scenario_P01_adaptive_solo_20261009_040950.mat'), ...
         fullfile(r,'scenario_P01_adaptive_collaborative_20261009_041509','scenario_P01_adaptive_collaborative_20261009_041509.mat')};
plotScenariosHRC_compact(files, struct('out', here));
