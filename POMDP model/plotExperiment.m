function data = plotExperiment(name, resultsDir)
% plotExperiment  Load and plot one live run saved by experiment_logger.py
%
%   data = plotExperiment('solo_experiment_1')
%   data = plotExperiment('assistive_experiment_2', 'C:\myproject\results')
%
%   Reads <name>.csv (one row per step) and <name>.json (run info + events),
%   plots every numeric signal against time with the events marked as
%   vertical lines, and saves the figure as a vector PDF for the paper.
%
%   Output struct:
%     data.table   all logged steps (MATLAB table, column t = time in s)
%     data.info    run information (mode, start time, duration, ...)
%     data.events  struct array of events (t, step, event, ...)

if nargin < 2
    resultsDir = 'results';
end

csvFile  = fullfile(resultsDir, [name '.csv']);
jsonFile = fullfile(resultsDir, [name '.json']);

% ---------- load ----------
T = readtable(csvFile, 'TextType', 'string');
J = jsondecode(fileread(jsonFile));

events = J.events;
if iscell(events)                 % events with different fields -> cell array
    events = [events{:}];
end

data.table  = T;
data.info   = J.info;
data.events = events;

% ---------- choose signals to plot ----------
skip = {'t', 'step', 'extra'};
names = T.Properties.VariableNames;
isNum = varfun(@isnumeric, T, 'OutputFormat', 'uniform');
plotVars = names(isNum & ~ismember(names, skip));

if isempty(plotVars)
    warning('No numeric columns to plot in %s', csvFile);
    return
end

% ---------- plot ----------
nV  = numel(plotVars);
fig = figure('Name', name, 'Color', 'w', ...
             'Units', 'inches', 'Position', [1 1 7.16 1.6*nV]);
tl  = tiledlayout(fig, nV, 1, 'TileSpacing', 'compact', 'Padding', 'compact');

for k = 1:nV
    ax = nexttile(tl);
    plot(ax, T.t, T.(plotVars{k}), 'LineWidth', 1.2);
    hold(ax, 'on'); grid(ax, 'on'); box(ax, 'on');
    ylabel(ax, strrep(plotVars{k}, '_', ' '), 'FontSize', 9);
    set(ax, 'FontSize', 9);

    % mark events (skip start/stop to keep it clean)
    for e = 1:numel(events)
        if any(strcmp(events(e).event, {'start', 'stop'}))
            continue
        end
        xline(ax, events(e).t, '--', strrep(events(e).event, '_', ' '), ...
              'FontSize', 7, 'LabelVerticalAlignment', 'bottom', ...
              'Color', [0.4 0.4 0.4]);
    end

    if k < nV
        set(ax, 'XTickLabel', []);
    end
    xlim(ax, [T.t(1) T.t(end)]);
end
xlabel(tl, 'Time (s)', 'FontSize', 10);
title(tl, strrep(name, '_', ' '), 'FontSize', 11);

% ---------- save as vector PDF (IEEE-ready) ----------
exportgraphics(fig, fullfile(resultsDir, [name '.pdf']), 'ContentType', 'vector');
fprintf('Saved %s\n', fullfile(resultsDir, [name '.pdf']));

% ---------- print the event log ----------
fprintf('\nEvents in %s (duration %.1f s):\n', name, data.info.duration_s);
for e = 1:numel(events)
    fprintf('  %8.2f s   %s\n', events(e).t, events(e).event);
end
end
