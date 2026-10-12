function pw = fatigueWarningPopupHRC(pw, status, mode, mainFig)
% FATIGUE WARNING POP-UP for the worker (liveHRC, every decision epoch).
% A separate, large, always-on-top window while the fatigue dialogue of the
% monitor (humanStateMonitorHRC) is active:
%   dialog 'warning'  "You seem tired" - collaborative mode: the robot has stopped
%                     ENTER (or the button) = continue working (RESUME)
%                     P     (or the button) = take a rest (PAUSE): the robot goes
%                                             to its safe position
%   dialog 'rest'     "Rest" - ENTER = I am back, continue (RESUME)
% The answer is passed to liveHRC exactly like the keys R / P of the main
% window (appdata 'keys' of mainFig), so the decision layer is unchanged.
% The pop-up closes itself when the dialogue ends; it cannot be closed by
% hand while a warning is open (it must be answered).
%
% pw      : pop-up memory ([] at the start)
% status  : monitor status (status.dialog, status.level, status.fatigueType, status.message)
% mode    : task mode (0 solo, 1 assistive, 2 collaborative)
% mainFig : figure that collects the keys of liveHRC
if isempty(pw), pw = struct('fig', [], 'dialog', 'none'); end
dg = status.dialog;
active = any(strcmp(dg, {'warning', 'rest'}));
if ~active
    if ~isempty(pw.fig) && isvalid(pw.fig), delete(pw.fig); end
    pw.fig = []; pw.dialog = dg;
    return
end
isNew = isempty(pw.fig) || ~isvalid(pw.fig);
if isNew
    pw.fig = figure('Name', 'HRC - FATIGUE WARNING', 'NumberTitle', 'off', 'MenuBar', 'none', ...
        'ToolBar', 'none', 'Units', 'normalized', 'Position', [0.22 0.25 0.56 0.5], 'Resize', 'off', ...
        'Color', [1 0.93 0.75], 'CloseRequestFcn', @(src, e) [], 'HandleVisibility', 'off', ...
        'WindowKeyPressFcn', @(src, e) onKey(e.Key, mainFig));
    pw.title = uicontrol(pw.fig, 'Style', 'text', 'Units', 'normalized', 'Position', [0.04 0.66 0.92 0.26], ...
        'FontSize', 26, 'FontWeight', 'bold', 'BackgroundColor', [1 0.93 0.75], 'ForegroundColor', [0.7 0 0]);
    pw.text = uicontrol(pw.fig, 'Style', 'text', 'Units', 'normalized', 'Position', [0.04 0.36 0.92 0.30], ...
        'FontSize', 15, 'BackgroundColor', [1 0.93 0.75], 'ForegroundColor', [0.15 0.15 0.15]);
    pw.bGo = uicontrol(pw.fig, 'Style', 'pushbutton', 'Units', 'normalized', 'Position', [0.06 0.07 0.42 0.22], ...
        'FontSize', 15, 'FontWeight', 'bold', 'BackgroundColor', [0.80 0.93 0.80], ...
        'Callback', @(src, e) onKey('return', mainFig));
    pw.bRest = uicontrol(pw.fig, 'Style', 'pushbutton', 'Units', 'normalized', 'Position', [0.52 0.07 0.42 0.22], ...
        'FontSize', 15, 'FontWeight', 'bold', 'BackgroundColor', [0.85 0.88 1.0], ...
        'Callback', @(src, e) onKey('p', mainFig));
    try, pw.fig.WindowStyle = 'alwaysontop'; catch, end   % R2023b+ for figure(); ignored otherwise
    beep;
end
if isNew || ~strcmp(dg, pw.dialog)                       % new state: texts and buttons
    if strcmp(dg, 'warning')
        set(pw.title, 'String', 'YOU SEEM TIRED');
        if mode == 2
            msg = sprintf(['Your %s fatigue is HIGH.\nThe robot has STOPPED.\n' ...
                'Please take a rest, or continue the task (the robot will work slowly).'], status.fatigueType);
        else
            msg = sprintf('Your %s fatigue is HIGH.\nPlease consider taking a rest.', status.fatigueType);
        end
        set(pw.text, 'String', msg);
        set(pw.bGo, 'String', 'ENTER  -  continue working', 'Visible', 'on');
        set(pw.bRest, 'String', 'P  -  take a rest', 'Visible', 'on');
    else
        set(pw.title, 'String', 'REST');
        set(pw.text, 'String', sprintf(['The robot waits at its safe position.\n' ...
            'Come back to the workstation, then press ENTER to continue.\n\n%s'], status.message));
        set(pw.bGo, 'String', 'ENTER  -  I am back, continue', 'Visible', 'on');
        set(pw.bRest, 'Visible', 'off');
        beep;
    end
    figure(pw.fig);                                     % bring it to the front once per new state
end
pw.dialog = dg;
end

function onKey(key, mainFig)
% answer -> the key queue of liveHRC (same as R / P in the main window)
if ~isvalid(mainFig), return; end
switch lower(key)
    case {'return', 'enter', 'r'}
        k = 'return';
    case 'p'
        k = 'p';
    otherwise
        return
end
setappdata(mainFig, 'keys', [getappdata(mainFig, 'keys') {k}]);
end
