function act = actionDefinitionHRC()
% Action set of the hierarchical POMDP HRC model (identifiers unchanged)
%  1 SOLO_CONTINUE_SEPARATED            (sc)
%  2 SOLO_SLOW_SEPARATED                (ss)
%  3 APPROACH_ASSISTIVE_ZONE            (ap)
%  4 ASSISTIVE_HOLD_CONFIRM             (ah)  stopped
%  5 COLLABORATIVE_NORMAL               (cn)
%  6 COLLABORATIVE_SLOW_SMOOTH          (cs)
%  7 COLLABORATIVE_HOLD_CONFIRM         (ch)  stopped, also the collaborative WAIT
%  8 ROBOT_ASSIST_OR_TASK_REALLOCATION  (ra)
%  9 RETREAT_TO_SEPARATED_ZONE          (rt)
% 10 PROTECTIVE_STOP                    (ps)  stopped
% Code 0 is NOT an action: it marks the task-layer transfer phase in the logs.
act.names = {'SOLO_CONTINUE_SEPARATED','SOLO_SLOW_SEPARATED','APPROACH_ASSISTIVE_ZONE', ...
    'ASSISTIVE_HOLD_CONFIRM','COLLABORATIVE_NORMAL','COLLABORATIVE_SLOW_SMOOTH', ...
    'COLLABORATIVE_HOLD_CONFIRM','ROBOT_ASSIST_OR_TASK_REALLOCATION', ...
    'RETREAT_TO_SEPARATED_ZONE','PROTECTIVE_STOP'};
act.short = {'sc','ss','ap','ah','cn','cs','ch','ra','rt','ps'};
%              1  2  3  4  5  6  7  8  9 10
act.motion = [2  1  1  0  2  1  0  1  1  0];   % commanded robot motion after the action
% hand-zone band the robot motion aims at (-1 = robot keeps its position):
% approach moves the robot so that the palm ends in the assistive band,
% retreat so that it ends in the separated band. No collaborative action
% changes the robot position, so the collaborative distance is kept.
act.target = [-1 -1 2 -1 -1 -1 -1 -1 3 -1];
act.solo   = [1 2 8 9 10];          % action set of solo mode
act.assist = [3 4 6 8 9 10];        % action set of assistive mode
act.collab = [5 6 7 8 10];          % action set of collaborative mode
act.collabExecution = [5 6 8];      % actions that need the palm inside the collaborative envelope
act.pause = [4 7 10];
act.confirm = [4 7];
% allowed{M+1}(a, Z+1) = 1 if action a may be used in mode M when the HAND
% is in zone Z (columns: prohibited, collaborative, assistive, separated)
act.allowed = cell(1,3);
%                   Z: 0 1 2 3
act.allowed{1} = [0 0 0 1;    % 1  solo continue: only with the hand separated
                  0 1 1 1;    % 2  solo slow
                  0 0 0 0;    % 3
                  0 0 0 0;    % 4
                  0 0 0 0;    % 5
                  0 0 0 0;    % 6
                  0 0 0 0;    % 7
                  0 0 0 1;    % 8  reallocation
                  0 1 1 1;    % 9  retreat
                  1 1 1 1];   % 10 protective stop
act.allowed{2} = [0 0 0 0;    % 1
                  0 0 0 0;    % 2
                  0 0 0 1;    % 3  approach only from the separated band
                  0 1 1 1;    % 4  hold (stopped wait)
                  0 0 0 0;    % 5
                  0 1 1 0;    % 6  slow and smooth near the hand
                  0 0 0 0;    % 7
                  0 0 1 0;    % 8  assistance only in the assistive region
                  0 1 1 1;    % 9  retreat
                  1 1 1 1];   % 10 protective stop
act.allowed{3} = [0 0 0 0;    % 1
                  0 0 0 0;    % 2
                  0 0 0 0;    % 3  no approach in collaborative mode
                  0 0 0 0;    % 4
                  0 1 0 0;    % 5  collaborative execution needs the palm in the envelope
                  0 1 0 0;    % 6
                  0 1 1 1;    % 7  hold / wait (stopped), never chases the hand
                  0 1 0 0;    % 8  assistance / reallocation needs the palm in the envelope
                  0 0 0 0;    % 9  no retreat in collaborative mode
                  1 1 1 1];   % 10 protective stop
