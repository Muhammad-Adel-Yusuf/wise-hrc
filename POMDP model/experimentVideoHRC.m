function vid = experimentVideoHRC(cmd, vid, varargin)
% Video of a live experiment (liveHRC). MATLAB does NOT capture its window itself (getframe of the large live window
% took ~3 s and slowed the 0.5 s decision loop): the camera program (main.py, fatigue_monitor/video_record.py)
% records the camera AND a screen capture of the windows whose title starts with "HRC live", while
% videos\recording.json says so. MATLAB only writes one status row per decision and starts / stops the recording.
%
% vid = experimentVideoHRC('start', root, part)      new session folder root\session_<date>_<time>[_partN],
%                                                    status.csv; recording.json -> main.py starts recording
% vid = experimentVideoHRC('frame', vid, figs, row)  one status row (each decision epoch); figs is not used
% vid = experimentVideoHRC('stop',  vid, composer)   close, recording.json -> main.py stops, then start the composer
%                                                    (make_experiment_video.py) in the background -> experiment_video.mp4
% The live window must stay VISIBLE on the screen (not minimised or covered) to be in the video.
STATUS_COLS = {'t_wall','t_s','session','mode','phase','final','final_name','planner','robot_action', ...
    'Fm','Fp','attention','level','dialog','zone','risk','hand_m','clearance_m','head_m','verified', ...
    'supervisor','message'};
switch cmd
    case 'start'
        root = vid; part = varargin{1};
        if ~exist(root, 'dir'), mkdir(root); end
        name = ['session_' char(datetime('now','Format','yyyyMMdd_HHmmss'))];
        if part > 1, name = sprintf('%s_part%d', name, part); end
        d = fullfile(root, name);
        mkdir(d);
        fs = fopen(fullfile(d, 'status.csv'), 'w'); fprintf(fs, '%s\n', strjoin(STATUS_COLS, ','));
        vid = struct('dir', d, 'root', root, 'fs', fs, 'n', 0, 'cols', {STATUS_COLS});
        control(root, true, d);
        fprintf('VIDEO: recording started -> %s (camera + this window, recorded by main.py)\n', d);
    case 'frame'
        row = varargin{2};
        vals = cell(1, numel(vid.cols));
        for i = 1:numel(vid.cols)
            v = row.(vid.cols{i});
            if isnumeric(v) || islogical(v)
                vals{i} = num2str(double(v), '%.15g');
            else
                vals{i} = ['"' strrep(char(v), '"', '''') '"'];
            end
        end
        fprintf(vid.fs, '%s\n', strjoin(vals, ','));
        vid.n = vid.n + 1;
    case 'stop'
        if isempty(vid), return; end
        composer = varargin{1};
        fclose(vid.fs);
        control(vid.root, false, vid.dir);
        fprintf('VIDEO: recording stopped (%d decisions, %.0f s) -> %s\n', vid.n, vid.n / 2, vid.dir);
        if vid.n >= 2 && exist(composer, 'file')
            system(sprintf('start "experiment video" /min python "%s" "%s"', composer, vid.dir));
            fprintf('VIDEO: making experiment_video.mp4 in the background (a minimised window)\n');
        end
        vid = [];
end
end

function control(root, on, d)
% recording.json for main.py (temp file + move: never read half written)
f = fullfile(root, 'recording.json');
fid = fopen([f '.tmp'], 'w');
fprintf(fid, '{"recording": %s, "dir": "%s", "window": "HRC live"}\n', mat2str(on), strrep(d, '\', '\\'));
fclose(fid);
movefile([f '.tmp'], f, 'f');
end
