% main.m  Simulate the quadrotor on one trajectory or on all nine.
%
%   Set cfg.trajectory to 1 ... 9 for a single run (animation, plots, figures saved),
%   or to 'all' to run every trajectory, save all figures and write the summary table.
%   Figures and tables go to report/figures, which the LaTeX report reads directly.

clear; close all; clc;
addpath(fullfile(fileparts(mfilename('fullpath')), 'src'));

cfg = defaultSettings();
cfg.trajectory   = 'all';       % 1 ... 9, or 'all'
cfg.drag.enabled = false;   % aerodynamic drag on/off
cfg.layout       = 1;       % animation side panels: 1 or 2
cfg.animate      = true;    % single trajectory only
cfg.showPlots    = true;    % single trajectory only
cfg.saveFigures  = true;    % write figures and tables to cfg.figureDir

runScenarios(cfg);
