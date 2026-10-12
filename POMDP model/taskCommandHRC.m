function tc = taskCommandHRC(mode, instruction)
% TASK-LAYER INTERFACE: the command a task layer sends to the framework at
% every decision epoch (input of hrcStepHRC). This file creates a command
% with default values; a task layer (taskLayer of a test script now, any
% task-allocation method later) fills in the fields it knows.
%
% tc = taskCommandHRC(mode, instruction)
%
% ---------------- used by the framework now
%   mode            task mode M: 0 solo, 1 assistive, 2 collaborative
%   instruction     current assembly instruction I (1..6)
%   safePose        robot pose used while the worker rests [m, frame W]
%   transferTarget  [] = the planner decides; a pose = task-layer motion to
%                   this pose (fetch, resume after a rest, ...), action code 0
%   allowedActions  [] = all actions of the mode; 1x10 0/1 mask otherwise
%                   (e.g. only hold / stop during a handover)
%   handoverActive  1 while the robot holds a tool at the handover point
%   toolTaken       1 when the camera / keyboard confirms the tool was taken
%   allowReallocation  0 (default): the robot never takes over the human's
%                   work by itself (action ra in solo / collaborative mode
%                   is replaced by normal work). A future allocation layer
%                   sets it to 1 after the worker accepted an offer.
% ---------------- reserved for a future task-allocation layer
%   (stored and passed to the status report, not used by the planner yet)
%   taskType        'robot' | 'human' | 'either' | 'joint'
%   needHelp        0/1 the worker needs the robot's help in this task
%   physicalDemand  0 low, 1 medium, 2 high
%   mentalDemand    0 low, 1 medium, 2 high
%
% The framework answers with a status report (field status of the output
% of hrcStepHRC, see humanStateMonitorHRC): fatigue level, rest warning,
% rest active, "help would be useful", robot action and stop reason.
if nargin < 1, mode = 0; end
if nargin < 2, instruction = 1; end
prm = stateDefinitionHRC();
types = {'either','human','joint'};
tc.mode = mode;
tc.instruction = instruction;
tc.safePose = prm.geo.poseSolo;
tc.transferTarget = [];
tc.allowedActions = [];
tc.handoverActive = 0;
tc.toolTaken = NaN;
tc.allowReallocation = 0;
tc.taskType = types{mode+1};
tc.needHelp = prm.assistNeeded(instruction);
tc.physicalDemand = 1;
tc.mentalDemand = 1;
end
