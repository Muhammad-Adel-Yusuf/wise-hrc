function v = robotSpeedsHRC()
% Commanded joint speed [deg/s] of every robot action file, index n+1 for action n = 0..11:
% the 'speed=' of tasks/task<n>.txt (slow / medium / normal / fast or a number) with the names and the
% speed cap of robot_code/robot_code.py (SPEEDS, SPEED_CAP), so MATLAB plots exactly what the robot program
% commands. Stop / hold actions (stop=true) are 0. Read once (persistent).
persistent cache
if ~isempty(cache), v = cache; return; end
root = fullfile(fileparts(mfilename('fullpath')), '..');
names = struct('slow',5,'medium',15,'normal',23,'fast',30);
cap = Inf;
try
    src = fileread(fullfile(root, 'robot_code', 'robot_code.py'));
    tok = regexp(src, 'SPEEDS\s*=\s*\{([^}]*)\}', 'tokens', 'once');
    if ~isempty(tok)
        kv = regexp(tok{1}, '''(\w+)''\s*:\s*([\d.]+)', 'tokens');
        for i = 1:numel(kv), names.(kv{i}{1}) = str2double(kv{i}{2}); end
    end
    tok = regexp(src, '(?m)^SPEED_CAP\s*=\s*([\d.]+)', 'tokens', 'once');
    if ~isempty(tok), cap = str2double(tok{1}); end
catch
end
v = zeros(1, 12);
for n = 0:11
    f = fullfile(root, 'tasks', sprintf('task%d.txt', n));
    if ~isfile(f), continue; end
    txt = fileread(f);
    if ~isempty(regexp(txt, '(?m)^\s*stop\s*=\s*(true|1)', 'once')), continue; end
    tok = regexp(txt, '(?m)^\s*speed\s*=\s*(\S+)', 'tokens', 'once');
    s = names.slow;
    if ~isempty(tok)
        if isfield(names, lower(tok{1})), s = names.(lower(tok{1})); else, s = str2double(tok{1}); end
    end
    v(n + 1) = min(s, cap);
end
cache = v;
end
