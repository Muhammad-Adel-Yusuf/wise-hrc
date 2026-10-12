function [m, ev] = scenarioReportHRC(in, prm, opts)
% Saves (or re-makes) everything of one recorded scenario for the paper.
%
% scenarioReportHRC(rec, prm, opts)  called by liveHRC when you press X
% scenarioReportHRC(matFile)         later, WITHOUT a new experiment: loads
%                                    results\scenario_...\scenario_....mat,
%                                    recomputes the metrics and re-draws the figure
% scenarioReportHRC(matFile, [], opts)   e.g. opts.width = 7.16 (double column)
%
% Folder results\scenario_<mode>_<date>\ (next to the .mat when re-made):
%   scenario_<name>.pdf / .png / .fig   journal figure (sessionFigureHRC)
%   scenario_<name>.mat                 rec (all recorded values), prm, metrics m, events ev
%   scenario_<name>_metrics.csv         the numbers for the paper (scenarioMetricsHRC)
%   scenario_<name>_events.csv          events and robot actions in time order
%   scenario_<name>_data.csv            every decision epoch (0.5 s): all values,
%                                       supervisor reason, fatigue adaptation, task phase
% opts: width (in, default 3.5), height (default 4.6)
if nargin < 3, opts = struct(); end
if ischar(in) || isstring(in)
    S = load(in);
    rec = S.rec; prm = S.prm;
    outDir = fileparts(char(in));
    if isempty(outDir), outDir = pwd; end
else
    rec = in;
    outDir = fullfile(pwd, 'results', ['scenario_' rec.name]);
end
if ~exist(outDir, 'dir'), mkdir(outDir); end
base = fullfile(outDir, ['scenario_' rec.name]);
[m, ev] = scenarioMetricsHRC(rec, prm);
save([base '.mat'], 'rec', 'prm', 'm', 'ev');
% ---------------- metrics: one row per value
f = fieldnames(m);
val = cell(numel(f),1);
for i = 1:numel(f)
    v = m.(f{i});
    if ischar(v), val{i} = v; else, val{i} = sprintf('%.4g', v); end
end
writetable(table(f, val, 'VariableNames', {'metric','value'}), [base '_metrics.csv']);
writetable(ev, [base '_events.csv']);
% ---------------- every epoch
T = array2table(rec.data, 'VariableNames', fieldnames(rec.cols)');
for fld = {'reason','adapt','phase'}
    if isfield(rec, fld{1}) && numel(rec.(fld{1}))==height(T)
        T.(fld{1}) = rec.(fld{1})(:);
    end
end
writetable(T, [base '_data.csv']);
% ---------------- figure
w = 3.5; h = 4.6;
if isfield(opts,'width') && ~isempty(opts.width), w = opts.width; end
if isfield(opts,'height') && ~isempty(opts.height), h = opts.height; end
sessionFigureHRC(rec, prm, struct('width',w,'height',h,'file',base));
% ---------------- short summary in the command window
fprintf('\nscenario %s saved in %s\n', rec.name, outDir);
fprintf('  %.0f s | normal %.0f%%, slow %.0f%%, stopped %.0f%%, protective stop %.0f%%\n', ...
    m.duration_s, m.share_normal_pct, m.share_slow_pct, m.share_stopped_pct, m.share_protective_stop_pct);
fprintf('  protective stops %d (proximity %d, sensing %d, distraction %d) | min hand-tool distance %.2f m\n', ...
    m.stops_total, m.stops_proximity, m.stops_sensing, m.stops_distraction, m.min_hand_tool_dist_m);
fprintf('  safety checks (should be 0): moving with hand in prohibited %d, normal speed with hand close %d\n', ...
    m.check_moving_in_prohibited_epochs, m.check_normal_speed_hand_close_epochs);
fprintf('  geometry verified %.0f%% | camera %.1f msg/s, age %.2f s | Fm available %.0f%%\n', ...
    m.geometry_verified_pct, m.camera_msg_rate_hz, m.camera_age_mean_s, m.Fm_available_pct);
end
