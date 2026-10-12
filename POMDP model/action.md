# Robot commands - live HRC session

Started: 2026-10-09 03:56:28  
One row per decision epoch (0.5 s). The robot executes the **code** of the LAST row.

## Codes

| code | short | action | speed |
|---|---|---|---|
| 0 | tr | task-layer motion to the target pose (tool fetch, go to / come back from the safe position) | 0.3 |
| 1 | sc | SOLO_CONTINUE_SEPARATED | 1.0 |
| 2 | ss | SOLO_SLOW_SEPARATED | 0.3 |
| 3 | ap | APPROACH_ASSISTIVE_ZONE | 0.3 |
| 4 | ah | ASSISTIVE_HOLD_CONFIRM | 0.0 |
| 5 | cn | COLLABORATIVE_NORMAL | 1.0 |
| 6 | cs | COLLABORATIVE_SLOW_SMOOTH | 0.3 |
| 7 | ch | COLLABORATIVE_HOLD_CONFIRM | 0.0 |
| 8 | ra | ROBOT_ASSIST_OR_TASK_REALLOCATION | 0.3 |
| 9 | rt | RETREAT_TO_SEPARATED_ZONE | 0.3 |
| 10 | ps | PROTECTIVE_STOP | 0.0 |
| 11 | wt | wait at safe position (retreat 9 while already at the safe position) | 0.0 |

speed: fraction of normal robot speed (1 normal, 0.3 slow, 0 stand still).  
source: planner = POMDP decision executed as it is; fatigue = changed by fatigueAdaptationHRC; safety = changed by safetySupervisorHRC; task = task-layer motion (code 0); session = session stopped.

## Commands

| step | t_s | mode | session | planner | adapted | supervisor | code | short | speed | source | target_xyz_m | reason |
|---|---|---|---|---|---|---|---|---|---|---|---|---|
| 1 | 13.64 | assistive | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 2 | 17.16 | assistive | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 3 | 19.30 | assistive | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 4 | 20.61 | assistive | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 5 | 21.86 | assistive | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 6 | 23.15 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 7 | 24.31 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 8 | 25.27 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 9 | 26.34 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 10 | 27.29 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 11 | 28.26 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 12 | 29.38 | collaborative | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.288 0.251 0.198 | transfer |
| 13 | 30.46 | collaborative | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.288 0.251 0.198 | transfer |
| 14 | 31.46 | collaborative | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.288 0.251 0.198 | transfer |
| 15 | 32.54 | collaborative | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.288 0.251 0.198 | transfer |
| 16 | 33.59 | collaborative | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.288 0.251 0.198 | transfer |
| 17 | 34.69 | collaborative | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.288 0.251 0.198 | transfer |
| 18 | 35.65 | collaborative | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.288 0.251 0.198 | transfer |
| 19 | 36.62 | collaborative | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.288 0.251 0.198 | transfer |
| 20 | 37.62 | collaborative | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.288 0.251 0.198 | transfer |
| 21 | 38.77 | collaborative | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.288 0.251 0.198 | transfer |
| 22 | 40.31 | collaborative | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.288 0.251 0.198 | transfer |
| 23 | 41.19 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 24 | 42.31 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 25 | 43.26 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 26 | 44.34 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 27 | 45.44 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 28 | 46.41 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 29 | 47.42 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 30 | 48.37 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 31 | 49.33 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 32 | 50.37 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 33 | 51.35 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 34 | 52.33 | collaborative | 1 | ps | ps | ps | 10 | ps | 0.0 | planner | - | active_interaction_risk |
| 35 | 54.05 | collaborative | 1 | ps | ps | ps | 10 | ps | 0.0 | planner | - | active_interaction_risk |
| 36 | 55.16 | collaborative | 1 | ps | ps | ps | 10 | ps | 0.0 | planner | - | active_interaction_risk |
| 37 | 56.13 | collaborative | 1 | cn | cn | ps | 10 | ps | 0.0 | safety | - | stop_held_until_verified_clearance |
| 38 | 57.13 | collaborative | 1 | cn | cn | ps | 10 | ps | 0.0 | safety | - | stop_held_until_verified_clearance |
| 39 | 58.24 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 40 | 59.96 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 41 | 61.09 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 42 | 62.07 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 43 | 62.95 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 44 | 63.97 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 45 | 64.94 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 46 | 66.09 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 47 | 67.01 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 48 | 68.03 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 49 | 68.94 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 50 | 69.89 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 51 | 70.92 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 52 | 71.89 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 53 | 72.80 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 54 | 73.92 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 55 | 74.97 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 56 | 75.93 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 57 | 76.90 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 58 | 77.83 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 59 | 78.68 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 60 | 79.76 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 61 | 80.82 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 62 | 81.82 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 63 | 83.09 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 64 | 84.31 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 65 | 85.71 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 66 | 86.67 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 67 | 87.84 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 68 | 89.01 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 69 | 90.02 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 70 | 91.18 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 71 | 92.15 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 72 | 93.36 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 73 | 94.79 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 74 | 96.15 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 75 | 97.48 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 76 | 98.73 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 77 | 99.86 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 78 | 101.03 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 79 | 102.09 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 80 | 103.10 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 81 | 104.13 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 82 | 105.17 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 83 | 106.22 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 84 | 107.28 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 85 | 108.34 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 86 | 109.35 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 87 | 110.97 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 88 | 112.06 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 89 | 112.97 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 90 | 114.07 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 91 | 115.09 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 92 | 116.09 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 93 | 117.28 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 94 | 118.34 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 95 | 119.50 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 96 | 120.42 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 97 | 121.93 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 98 | 122.93 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 99 | 123.83 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 100 | 124.91 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 101 | 125.91 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 102 | 127.09 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 103 | 128.19 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 104 | 129.25 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 105 | 130.36 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 106 | 131.44 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 107 | 132.70 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 108 | 133.78 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 109 | 134.76 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 110 | 135.72 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 111 | 136.77 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 112 | 137.81 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 113 | 138.77 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 114 | 139.97 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 115 | 141.04 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 116 | 142.03 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 117 | 143.07 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 118 | 144.05 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 119 | 145.05 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 120 | 146.00 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 121 | 147.32 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 122 | 148.36 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 123 | 149.41 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 124 | 150.51 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 125 | 151.63 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 126 | 152.69 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 127 | 153.77 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 128 | 154.81 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 129 | 155.73 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 130 | 156.75 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 131 | 157.92 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 132 | 158.85 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 133 | 159.83 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 134 | 160.82 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 135 | 161.75 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 136 | 162.88 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 137 | 164.34 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 138 | 165.97 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 139 | 167.18 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 140 | 168.23 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 141 | 169.19 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 142 | 170.20 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 143 | 171.18 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 144 | 172.15 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 145 | 173.04 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 146 | 174.09 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 147 | 175.10 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 148 | 176.00 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 149 | 177.08 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 150 | 178.06 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 151 | 179.03 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 152 | 179.98 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 153 | 180.98 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 154 | 182.00 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 155 | 182.92 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 156 | 183.82 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 157 | 184.88 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 158 | 185.89 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 159 | 186.82 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 160 | 187.89 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 161 | 188.83 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 162 | 190.62 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 163 | 191.77 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 164 | 192.66 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 165 | 193.64 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 166 | 194.94 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 167 | 196.10 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 168 | 197.27 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 169 | 198.30 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 170 | 199.29 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 171 | 200.29 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 172 | 201.33 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 173 | 202.76 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 174 | 203.71 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 175 | 204.68 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 176 | 205.86 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 177 | 207.01 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 178 | 208.16 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 179 | 209.20 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 180 | 210.31 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 181 | 211.49 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 182 | 212.49 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 183 | 213.53 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 184 | 214.44 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 185 | 215.36 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 186 | 216.45 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 187 | 217.35 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 188 | 218.32 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 189 | 219.35 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 190 | 220.46 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 191 | 221.40 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 192 | 222.89 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 193 | 223.82 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 194 | 224.89 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 195 | 225.85 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 196 | 226.76 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 197 | 227.73 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 198 | 228.72 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 199 | 229.66 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 200 | 230.67 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 201 | 231.64 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 202 | 232.57 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 203 | 233.66 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 204 | 234.57 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 205 | 235.63 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 206 | 236.55 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 207 | 237.55 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 208 | 238.57 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 209 | 239.44 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 210 | 240.44 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 211 | 241.39 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 212 | 242.31 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 213 | 243.30 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 214 | 244.26 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 215 | 245.29 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 216 | 246.45 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 217 | 247.50 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 218 | 248.51 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 219 | 249.56 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 220 | 250.49 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 221 | 251.56 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 222 | 252.54 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 223 | 253.46 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 224 | 254.52 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 225 | 255.53 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 226 | 256.53 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 227 | 257.77 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 228 | 258.72 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 229 | 260.05 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 230 | 261.40 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 231 | 262.96 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 232 | 264.57 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 233 | 265.65 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 234 | 266.53 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 235 | 267.54 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 236 | 268.55 | collaborative | 1 | cn | cn | cn | 5 | cn | 1.0 | planner | - |  |
| 237 | 269.60 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 238 | 270.81 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 239 | 271.91 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 240 | 272.82 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 241 | 274.48 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 242 | 275.49 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 243 | 276.43 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 244 | 277.35 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 245 | 278.44 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 246 | 279.40 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 247 | 280.31 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 248 | 281.37 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 249 | 282.21 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 250 | 283.25 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 251 | 284.21 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 252 | 285.16 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 253 | 286.15 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 254 | 287.18 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 255 | 288.11 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 256 | 289.02 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 257 | 289.98 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 258 | 290.92 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 259 | 291.92 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 260 | 292.88 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 261 | 293.80 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 262 | 294.67 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 263 | 295.54 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 264 | 296.76 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 265 | 297.67 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 266 | 298.69 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 267 | 299.64 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 268 | 300.54 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 269 | 301.54 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 270 | 302.55 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 271 | 303.42 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 272 | 304.38 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 273 | 305.42 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 274 | 306.34 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 275 | 307.35 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 276 | 308.37 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 277 | 309.32 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 278 | 310.32 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 279 | 311.29 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 280 | 312.30 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 281 | 313.23 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 282 | 314.23 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 283 | 315.75 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 284 | 316.99 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 285 | 318.24 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 286 | 319.31 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 287 | 320.28 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 288 | 321.21 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 289 | 322.25 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 290 | 323.15 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 291 | 324.11 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 292 | 325.00 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 293 | 326.20 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 294 | 327.26 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 295 | 328.31 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 296 | 329.30 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 297 | 330.27 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 298 | 331.40 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 299 | 332.46 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 300 | 333.46 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 301 | 334.80 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 302 | 336.67 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 303 | 337.77 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 304 | 338.81 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 305 | 339.91 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 306 | 341.04 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 307 | 342.30 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 308 | 343.43 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 309 | 344.83 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 310 | 346.31 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 311 | 347.44 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 312 | 348.40 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 313 | 349.37 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 314 | 350.60 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 315 | 351.58 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 316 | 352.61 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 317 | 353.63 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 318 | 354.60 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 319 | 355.58 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 320 | 356.63 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 321 | 357.57 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 322 | 358.73 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 323 | 359.67 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 324 | 360.64 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 325 | 361.75 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 326 | 362.91 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 327 | 363.91 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 328 | 364.85 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 329 | 365.96 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 330 | 367.07 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 331 | 368.20 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 332 | 369.22 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 333 | 370.29 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 334 | 371.45 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 335 | 372.62 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 336 | 373.67 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 337 | 374.74 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 338 | 375.90 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 339 | 377.23 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 340 | 378.61 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 341 | 380.80 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 342 | 382.05 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 343 | 383.10 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 344 | 384.23 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 345 | 385.28 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 346 | 386.35 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 347 | 387.32 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 348 | 388.37 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 349 | 389.84 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 350 | 391.16 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 351 | 392.11 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 352 | 393.13 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 353 | 394.09 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 354 | 395.13 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 355 | 396.16 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 356 | 397.22 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 357 | 398.34 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 358 | 399.66 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 359 | 400.79 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 360 | 401.87 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 361 | 403.06 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 362 | 404.47 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 363 | 405.76 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 364 | 406.95 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 365 | 408.04 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 366 | 409.07 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 367 | 410.13 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 368 | 411.34 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 369 | 412.68 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 370 | 413.63 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 371 | 414.61 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 372 | 415.62 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 373 | 416.51 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 374 | 417.58 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 375 | 418.55 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 376 | 419.62 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 377 | 420.52 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 378 | 421.49 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 379 | 422.56 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 380 | 423.48 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 381 | 424.55 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 382 | 425.52 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 383 | 426.43 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 384 | 427.51 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 385 | 428.53 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 386 | 429.61 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 387 | 430.67 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 388 | 431.66 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 389 | 432.68 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 390 | 433.89 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 391 | 435.24 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 392 | 436.75 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | risky_distraction |
| 393 | 438.00 | collaborative | 1 | cs | cs | ps | 10 | ps | 0.0 | safety | - | stop_held_until_verified_clearance |
| 394 | 439.24 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 395 | 440.46 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 396 | 441.59 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 397 | 443.56 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 398 | 444.93 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 399 | 446.15 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 400 | 447.14 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 401 | 448.10 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 402 | 449.20 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 403 | 450.22 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 404 | 451.19 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 405 | 452.38 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 406 | 453.35 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 407 | 454.66 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 408 | 456.10 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 409 | 457.26 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 410 | 458.48 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 411 | 459.63 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 412 | 460.60 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 413 | 461.55 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 414 | 462.59 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 415 | 463.57 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 416 | 464.67 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 417 | 465.74 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 418 | 467.54 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 419 | 468.58 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 420 | 470.05 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 421 | 471.19 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 422 | 472.29 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 423 | 473.24 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 424 | 474.29 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 425 | 475.25 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 426 | 476.52 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 427 | 477.54 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 428 | 478.56 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 429 | 479.68 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 430 | 480.77 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 431 | 482.03 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 432 | 483.24 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 433 | 484.30 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 434 | 485.40 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 435 | 486.48 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 436 | 487.59 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 437 | 488.80 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 438 | 489.96 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 439 | 490.99 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 440 | 492.19 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 441 | 493.48 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 442 | 494.61 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 443 | 495.90 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 444 | 496.91 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 445 | 497.83 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 446 | 498.73 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 447 | 499.73 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 448 | 500.67 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 449 | 501.94 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 450 | 502.84 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 451 | 504.04 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 452 | 505.06 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 453 | 506.06 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 454 | 507.14 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 455 | 508.80 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 456 | 510.27 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 457 | 511.62 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 458 | 512.95 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 459 | 514.06 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 460 | 515.16 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 461 | 516.11 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 462 | 517.25 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 463 | 518.41 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 464 | 519.44 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 465 | 520.64 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 466 | 521.60 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 467 | 522.69 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 468 | 523.71 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 469 | 524.98 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 470 | 526.07 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 471 | 526.95 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 472 | 527.99 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 473 | 529.06 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 474 | 530.12 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 475 | 531.48 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 476 | 532.75 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 477 | 533.89 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 478 | 534.90 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 479 | 535.87 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 480 | 536.81 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 481 | 537.95 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 482 | 538.85 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 483 | 539.75 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 484 | 540.79 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 485 | 542.08 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 486 | 543.06 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 487 | 544.01 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 488 | 544.97 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 489 | 546.22 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 490 | 547.17 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 491 | 548.24 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 492 | 549.18 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 493 | 550.23 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 494 | 551.21 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 495 | 552.21 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 496 | 553.23 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 497 | 554.19 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 498 | 555.30 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 499 | 556.30 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 500 | 557.32 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 501 | 558.57 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 502 | 559.72 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 503 | 560.71 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 504 | 561.98 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 505 | 562.86 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 506 | 564.03 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 507 | 564.99 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 508 | 566.03 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 509 | 567.03 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 510 | 568.36 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 511 | 569.45 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 512 | 570.71 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 513 | 571.95 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 514 | 573.39 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 515 | 574.75 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 516 | 575.94 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 517 | 608.95 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 518 | 610.79 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 519 | 612.04 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 520 | 613.71 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 521 | 615.63 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 522 | 618.49 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 523 | 620.13 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 524 | 621.83 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 525 | 623.11 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 526 | 624.69 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 527 | 626.08 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 528 | 627.93 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 529 | 629.26 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 530 | 630.65 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 531 | 631.92 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 532 | 633.29 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 533 | 634.73 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 534 | 636.00 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 535 | 637.25 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 536 | 638.56 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 537 | 639.81 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 538 | 641.05 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 539 | 642.51 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 540 | 643.80 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 541 | 645.10 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 542 | 646.68 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 543 | 648.15 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 544 | 649.49 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 545 | 650.87 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 546 | 652.52 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 547 | 654.58 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 548 | 655.94 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 549 | 657.22 | assistive | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 550 | 658.65 | assistive | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 551 | 659.96 | assistive | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 552 | 661.17 | assistive | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 553 | 662.46 | assistive | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.077 0.002 0.419 | transfer |
| 554 | 663.86 | assistive | 1 | rt | rt | rt | 11 | wt | 0.0 | planner | - |  |
| 555 | 665.05 | assistive | 1 | rt | rt | rt | 11 | wt | 0.0 | planner | - |  |
| 556 | 666.51 | assistive | 1 | rt | rt | rt | 11 | wt | 0.0 | planner | - |  |
| 557 | 667.69 | assistive | 1 | rt | rt | rt | 11 | wt | 0.0 | planner | - |  |
| 558 | 668.98 | assistive | 1 | rt | rt | rt | 11 | wt | 0.0 | planner | - |  |
| 559 | 670.17 | assistive | 1 | rt | rt | rt | 11 | wt | 0.0 | planner | - |  |
| 560 | 671.43 | assistive | 1 | rt | rt | rt | 11 | wt | 0.0 | planner | - |  |
| 561 | 672.73 | assistive | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.158 -0.390 0.152 | transfer |
| 562 | 674.18 | assistive | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.158 -0.390 0.152 | transfer |
| 563 | 675.36 | assistive | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.158 -0.390 0.152 | transfer |
| 564 | 676.66 | assistive | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.158 -0.390 0.152 | transfer |
| 565 | 678.01 | assistive | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.158 -0.390 0.152 | transfer |
| 566 | 679.33 | assistive | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.158 -0.390 0.152 | transfer |
| 567 | 680.54 | assistive | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.158 -0.390 0.152 | transfer |
| 568 | 681.80 | assistive | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.158 -0.390 0.152 | transfer |
| 569 | 683.02 | assistive | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.158 -0.390 0.152 | transfer |
| 570 | 684.36 | assistive | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.158 -0.390 0.152 | transfer |
| 571 | 685.56 | assistive | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.158 -0.390 0.152 | transfer |
| 572 | 686.85 | assistive | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.158 -0.390 0.152 | transfer |
| 573 | 688.05 | assistive | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.158 -0.390 0.152 | transfer |
| 574 | 689.28 | assistive | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.158 -0.390 0.152 | transfer |
| 575 | 690.43 | assistive | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.281 0.147 0.417 | transfer |
| 576 | 691.90 | assistive | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.281 0.147 0.417 | transfer |
| 577 | 693.16 | assistive | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.281 0.147 0.417 | transfer |
| 578 | 694.41 | assistive | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.281 0.147 0.417 | transfer |
| 579 | 695.59 | assistive | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.281 0.147 0.417 | transfer |
| 580 | 696.73 | assistive | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.281 0.147 0.417 | transfer |
| 581 | 697.97 | assistive | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.281 0.147 0.417 | transfer |
| 582 | 699.28 | assistive | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.281 0.147 0.417 | transfer |
| 583 | 700.69 | assistive | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.281 0.147 0.417 | transfer |
| 584 | 702.02 | assistive | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.281 0.147 0.417 | transfer |
| 585 | 703.99 | assistive | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.281 0.147 0.417 | transfer |
| 586 | 705.69 | assistive | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.281 0.147 0.417 | transfer |
| 587 | 706.99 | assistive | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.281 0.147 0.417 | transfer |
| 588 | 708.22 | assistive | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.281 0.147 0.417 | transfer |
| 589 | 709.44 | assistive | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.281 0.147 0.417 | transfer |
| 590 | 710.71 | assistive | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.281 0.147 0.417 | transfer |
| 591 | 712.37 | assistive | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.281 0.147 0.417 | transfer |
| 592 | 713.68 | assistive | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.281 0.147 0.417 | transfer |
| 593 | 714.87 | assistive | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.281 0.147 0.417 | transfer |
| 594 | 716.09 | assistive | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.281 0.147 0.417 | transfer |
| 595 | 717.35 | assistive | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.281 0.147 0.417 | transfer |
| 596 | 718.92 | assistive | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.281 0.147 0.417 | transfer |
| 597 | 720.31 | assistive | 1 | ah | ah | ah | 4 | ah | 0.0 | planner | - |  |
| 598 | 721.70 | assistive | 1 | ah | ah | ah | 4 | ah | 0.0 | planner | - |  |
| 599 | 722.90 | assistive | 1 | ah | ah | ah | 4 | ah | 0.0 | planner | - |  |
| 600 | 724.12 | assistive | 1 | ah | ah | ah | 4 | ah | 0.0 | planner | - |  |
| 601 | 725.57 | assistive | 1 | ah | ah | ah | 4 | ah | 0.0 | planner | - |  |
| 602 | 726.80 | assistive | 1 | ah | ah | ah | 4 | ah | 0.0 | planner | - |  |
| 603 | 728.11 | assistive | 1 | ah | ah | ah | 4 | ah | 0.0 | planner | - |  |
| 604 | 729.25 | assistive | 1 | ah | ah | ah | 4 | ah | 0.0 | planner | - |  |
| 605 | 730.49 | assistive | 1 | ah | ah | ah | 4 | ah | 0.0 | planner | - |  |
| 606 | 732.12 | assistive | 1 | ah | ah | ah | 4 | ah | 0.0 | planner | - |  |
| 607 | 733.29 | assistive | 1 | ah | ah | ah | 4 | ah | 0.0 | planner | - |  |
| 608 | 734.60 | assistive | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.077 0.002 0.419 | transfer |
| 609 | 736.21 | assistive | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.077 0.002 0.419 | transfer |
| 610 | 737.48 | assistive | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.077 0.002 0.419 | transfer |
| 611 | 738.79 | assistive | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.077 0.002 0.419 | transfer |
| 612 | 740.14 | assistive | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.077 0.002 0.419 | transfer |
| 613 | 741.60 | assistive | 1 | rt | rt | rt | 11 | wt | 0.0 | planner | - |  |
| 614 | 743.01 | assistive | 1 | rt | rt | rt | 11 | wt | 0.0 | planner | - |  |
| 615 | 744.49 | assistive | 1 | rt | rt | rt | 11 | wt | 0.0 | planner | - |  |
| 616 | 745.87 | assistive | 1 | rt | rt | rt | 11 | wt | 0.0 | planner | - |  |
| 617 | 747.11 | assistive | 1 | rt | rt | rt | 11 | wt | 0.0 | planner | - |  |
| 618 | 748.43 | assistive | 1 | rt | rt | rt | 11 | wt | 0.0 | planner | - |  |
| 619 | 749.67 | assistive | 1 | rt | rt | rt | 11 | wt | 0.0 | planner | - |  |
| 620 | 751.08 | assistive | 1 | rt | rt | rt | 11 | wt | 0.0 | planner | - |  |
| 621 | 752.15 | assistive | 1 | rt | rt | rt | 11 | wt | 0.0 | planner | - |  |
| 622 | 753.49 | assistive | 1 | rt | rt | rt | 11 | wt | 0.0 | planner | - |  |
| 623 | 754.69 | assistive | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 624 | 788.72 | solo | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.077 0.002 0.419 | transfer |
| 625 | 791.01 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 626 | 792.57 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 627 | 793.91 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 628 | 795.42 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 629 | 796.81 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 630 | 798.39 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 631 | 800.26 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 632 | 801.83 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 633 | 803.35 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 634 | 804.86 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 635 | 806.29 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 636 | 808.02 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 637 | 809.44 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 638 | 811.20 | solo | 1 | ss | ss | ss | 2 | ss | 0.3 | planner | - |  |
| 639 | 812.65 | solo | 1 | ss | ss | ss | 2 | ss | 0.3 | planner | - |  |
| 640 | 814.15 | solo | 1 | ss | ss | ss | 2 | ss | 0.3 | planner | - |  |
| 641 | 815.60 | solo | 1 | ss | ss | ss | 2 | ss | 0.3 | planner | - |  |
| 642 | 817.09 | solo | 1 | sc | sc | ss | 2 | ss | 0.3 | safety | - | solo_separation_not_verified_slow |
| 643 | 818.63 | solo | 1 | sc | sc | ss | 2 | ss | 0.3 | safety | - | solo_separation_not_verified_slow |
| 644 | 820.12 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 645 | 821.57 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 646 | 823.30 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 647 | 824.78 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 648 | 826.24 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 649 | 827.59 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 650 | 828.99 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 651 | 830.26 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 652 | 831.79 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 653 | 833.40 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 654 | 834.97 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 655 | 836.48 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 656 | 837.87 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 657 | 839.31 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 658 | 840.72 | solo | 1 | ss | ss | ss | 2 | ss | 0.3 | planner | - |  |
| 659 | 842.38 | solo | 1 | ss | ss | ss | 2 | ss | 0.3 | planner | - |  |
| 660 | 844.05 | solo | 1 | sc | sc | ss | 2 | ss | 0.3 | safety | - | solo_separation_not_verified_slow |
| 661 | 846.01 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 662 | 847.61 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 663 | 849.13 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 664 | 851.06 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 665 | 852.66 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 666 | 854.17 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 667 | 855.66 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 668 | 857.13 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 669 | 858.91 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 670 | 860.56 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 671 | 862.06 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 672 | 863.47 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 673 | 864.91 | solo | 1 | ss | ss | ss | 2 | ss | 0.3 | planner | - |  |
| 674 | 866.59 | solo | 1 | ss | ss | ss | 2 | ss | 0.3 | planner | - |  |
| 675 | 868.14 | solo | 1 | ss | ss | ss | 2 | ss | 0.3 | planner | - |  |
| 676 | 869.86 | solo | 1 | ss | ss | ss | 2 | ss | 0.3 | planner | - |  |
| 677 | 871.34 | solo | 1 | sc | sc | ss | 2 | ss | 0.3 | safety | - | solo_separation_not_verified_slow |
| 678 | 873.02 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 679 | 874.65 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 680 | 876.16 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 681 | 877.64 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 682 | 879.16 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 683 | 880.85 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 684 | 882.41 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 685 | 883.85 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 686 | 885.38 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 687 | 886.82 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 688 | 888.29 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 689 | 889.78 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 690 | 891.30 | solo | 1 | sc | sc | ss | 2 | ss | 0.3 | safety | - | solo_separation_not_verified_slow |
| 691 | 892.83 | solo | 1 | ss | ss | ss | 2 | ss | 0.3 | planner | - |  |
| 692 | 894.36 | solo | 1 | sc | sc | ss | 2 | ss | 0.3 | safety | - | solo_separation_not_verified_slow |
| 693 | 896.00 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 694 | 897.58 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 695 | 899.03 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 696 | 900.33 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 697 | 901.66 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 698 | 903.04 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 699 | 905.15 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 700 | 906.68 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 701 | 907.95 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 702 | 909.15 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 703 | 910.44 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 704 | 911.58 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 705 | 912.79 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 706 | 913.95 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 707 | 915.25 | solo | 1 | ss | ss | ss | 2 | ss | 0.3 | planner | - |  |
| 708 | 916.69 | solo | 1 | ss | ss | ss | 2 | ss | 0.3 | planner | - |  |
| 709 | 918.32 | solo | 1 | ss | ss | ss | 2 | ss | 0.3 | planner | - |  |
| 710 | 920.03 | solo | 1 | ss | ss | ss | 2 | ss | 0.3 | planner | - |  |
| 711 | 921.74 | solo | 1 | ss | ss | ss | 2 | ss | 0.3 | planner | - |  |
| 712 | 923.04 | solo | 1 | ss | ss | ss | 2 | ss | 0.3 | planner | - |  |
| 713 | 924.28 | solo | 1 | sc | sc | ss | 2 | ss | 0.3 | safety | - | solo_separation_not_verified_slow |
| 714 | 925.60 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 715 | 926.95 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 716 | 928.25 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 717 | 929.57 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 718 | 930.89 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 719 | 932.03 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 720 | 933.53 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 721 | 935.14 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 722 | 936.36 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 723 | 937.56 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 724 | 938.69 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 725 | 939.99 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 726 | 941.31 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 727 | 942.50 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 728 | 943.71 | solo | 1 | sc | sc | ss | 2 | ss | 0.3 | safety | - | solo_separation_not_verified_slow |
| 729 | 945.08 | solo | 1 | ss | ss | ss | 2 | ss | 0.3 | planner | - |  |
| 730 | 946.36 | solo | 1 | sc | sc | ss | 2 | ss | 0.3 | safety | - | solo_separation_not_verified_slow |
| 731 | 947.68 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 732 | 949.06 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 733 | 950.27 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 734 | 951.45 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 735 | 952.70 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 736 | 953.80 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 737 | 955.07 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 738 | 956.39 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 739 | 957.56 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 740 | 959.49 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 741 | 960.92 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 742 | 962.22 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 743 | 963.53 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 744 | 964.80 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 745 | 966.03 | solo | 1 | ss | ss | ss | 2 | ss | 0.3 | planner | - |  |
| 746 | 967.46 | solo | 1 | ss | ss | ss | 2 | ss | 0.3 | planner | - |  |
| 747 | 968.99 | solo | 1 | ss | ss | ss | 2 | ss | 0.3 | planner | - |  |
| 748 | 970.43 | solo | 1 | sc | sc | ss | 2 | ss | 0.3 | safety | - | solo_separation_not_verified_slow |
| 749 | 972.14 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 750 | 973.63 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 751 | 975.23 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 752 | 976.58 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 753 | 977.82 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 754 | 979.14 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 755 | 980.51 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 756 | 981.82 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 757 | 983.06 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 758 | 984.44 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 759 | 985.76 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 760 | 987.65 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 761 | 989.29 | solo | 1 | sc | sc | sc | 1 | sc | 1.0 | planner | - |  |
| 762 | 990.88 | solo | 1 | ss | ss | ss | 2 | ss | 0.3 | planner | - |  |
| 763 | 992.53 | solo | 1 | ss | ss | ss | 2 | ss | 0.3 | planner | - |  |
| 764 | 994.01 | solo | 1 | ss | ss | ss | 2 | ss | 0.3 | planner | - |  |
| 765 | 995.33 | solo | 1 | ss | ss | ss | 2 | ss | 0.3 | planner | - |  |
| 766 | 996.72 | solo | 1 | ss | ss | ss | 2 | ss | 0.3 | planner | - |  |
| 767 | 998.26 | solo | 1 | sc | sc | ss | 2 | ss | 0.3 | safety | - | solo_separation_not_verified_slow |
| 768 | 999.66 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 769 | 1033.24 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 770 | 1034.90 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 771 | 1036.69 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 772 | 1038.19 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 773 | 1039.65 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 774 | 1041.19 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 775 | 1042.63 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 776 | 1044.17 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 777 | 1045.73 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 778 | 1047.17 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 779 | 1048.87 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 780 | 1050.32 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 781 | 1051.87 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 782 | 1053.86 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 783 | 1055.48 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 784 | 1057.06 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 785 | 1058.74 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 786 | 1060.22 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 787 | 1061.97 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 788 | 1063.64 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 789 | 1065.24 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 790 | 1066.83 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 791 | 1068.33 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 792 | 1069.92 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 793 | 1071.50 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 794 | 1072.91 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 795 | 1074.92 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 796 | 1076.57 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 797 | 1078.07 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 798 | 1079.46 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 799 | 1081.01 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 800 | 1082.58 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 801 | 1084.02 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 802 | 1085.43 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 803 | 1086.94 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 804 | 1088.45 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 805 | 1090.05 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 806 | 1091.77 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 807 | 1093.44 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 808 | 1095.10 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 809 | 1096.78 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 810 | 1098.21 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 811 | 1099.65 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 812 | 1100.98 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 813 | 1102.44 | solo | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 814 | 1103.90 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 815 | 1105.28 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 816 | 1106.65 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 817 | 1107.91 | collaborative | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.288 0.251 0.198 | transfer |
| 818 | 1109.49 | collaborative | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.288 0.251 0.198 | transfer |
| 819 | 1110.87 | collaborative | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.288 0.251 0.198 | transfer |
| 820 | 1112.07 | collaborative | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.288 0.251 0.198 | transfer |
| 821 | 1113.48 | collaborative | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.288 0.251 0.198 | transfer |
| 822 | 1114.72 | collaborative | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.288 0.251 0.198 | transfer |
| 823 | 1115.99 | collaborative | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.288 0.251 0.198 | transfer |
| 824 | 1117.30 | collaborative | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.288 0.251 0.198 | transfer |
| 825 | 1118.73 | collaborative | 1 | tr | tr | tr | 0 | tr | 0.3 | task | 0.288 0.251 0.198 | transfer |
| 826 | 1120.65 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 827 | 1122.16 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 828 | 1123.66 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 829 | 1125.07 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 830 | 1126.40 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 831 | 1127.65 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 832 | 1128.99 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 833 | 1130.35 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 834 | 1131.62 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 835 | 1132.76 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 836 | 1134.04 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 837 | 1135.27 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 838 | 1136.43 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 839 | 1137.74 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 840 | 1138.98 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 841 | 1140.44 | collaborative | 1 | ch | ch | ch | 7 | ch | 0.0 | planner | - |  |
| 842 | 1141.94 | collaborative | 1 | ch | ch | ch | 7 | ch | 0.0 | planner | - |  |
| 843 | 1143.23 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 844 | 1144.52 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 845 | 1145.80 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 846 | 1147.11 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 847 | 1148.34 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 848 | 1149.61 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 849 | 1150.87 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 850 | 1152.09 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 851 | 1153.48 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 852 | 1154.74 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 853 | 1156.00 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 854 | 1157.22 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 855 | 1158.42 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 856 | 1159.71 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 857 | 1161.10 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 858 | 1162.52 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 859 | 1163.91 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 860 | 1165.12 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 861 | 1166.27 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 862 | 1167.62 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 863 | 1168.91 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 864 | 1170.76 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 865 | 1172.83 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 866 | 1174.36 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 867 | 1175.57 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 868 | 1176.74 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 869 | 1178.12 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 870 | 1179.40 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 871 | 1180.64 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 872 | 1181.98 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 873 | 1183.24 | collaborative | 1 | ch | ch | ch | 7 | ch | 0.0 | planner | - |  |
| 874 | 1184.86 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 875 | 1186.32 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 876 | 1187.83 | collaborative | 1 | cs | cs | cs | 6 | cs | 0.3 | planner | - |  |
| 877 | 1189.42 | collaborative | 1 | cs | ch | ch | 7 | ch | 0.0 | fatigue | - | fatigue_warning_hold |
| 878 | 1193.19 | collaborative | 1 | cs | ch | ch | 7 | ch | 0.0 | fatigue | - | fatigue_warning_hold |
| 879 | 1194.66 | collaborative | 1 | cs | ch | ch | 7 | ch | 0.0 | fatigue | - | fatigue_warning_hold |
| 880 | 1196.04 | collaborative | 1 | cs | ch | ch | 7 | ch | 0.0 | fatigue | - | fatigue_warning_hold |
| 881 | 1197.45 | collaborative | 1 | cs | ch | ch | 7 | ch | 0.0 | fatigue | - | fatigue_warning_hold |
| 882 | 1199.09 | collaborative | 1 | cs | ch | ch | 7 | ch | 0.0 | fatigue | - | fatigue_warning_hold |
| 883 | 1200.66 | collaborative | 1 | cs | tr | tr | 0 | tr | 0.3 | task | 0.077 0.002 0.419 | rest_go_safe |
| 884 | 1202.50 | collaborative | 1 | cs | tr | tr | 0 | tr | 0.3 | task | 0.077 0.002 0.419 | rest_go_safe |
| 885 | 1203.35 | collaborative | 1 | ps | tr | ps | 10 | ps | 0.0 | safety | - | safety_unverified:stale_geometry |
| 886 | 1204.32 | collaborative | 1 | ps | tr | ps | 10 | ps | 0.0 | safety | - | safety_unverified:stale_geometry |
| 887 | 1205.17 | collaborative | 1 | ps | tr | ps | 10 | ps | 0.0 | safety | - | safety_unverified:stale_geometry |
| 888 | 1206.05 | collaborative | 1 | ps | tr | ps | 10 | ps | 0.0 | safety | - | safety_unverified:stale_geometry |
| 889 | 1207.09 | collaborative | 1 | cs | tr | ps | 10 | ps | 0.0 | safety | - | stop_held_until_verified_clearance |
| 890 | 1208.72 | collaborative | 1 | cs | tr | tr | 0 | tr | 0.3 | task | 0.077 0.002 0.419 | rest_go_safe |
| 891 | 1210.26 | collaborative | 1 | cs | tr | tr | 0 | tr | 0.3 | task | 0.077 0.002 0.419 | rest_go_safe |
| 892 | 1211.94 | collaborative | 1 | ch | tr | tr | 0 | tr | 0.3 | task | 0.077 0.002 0.419 | rest_go_safe |
| 893 | 1213.63 | collaborative | 1 | ch | tr | tr | 0 | tr | 0.3 | task | 0.077 0.002 0.419 | rest_go_safe |
| 894 | 1215.31 | collaborative | 1 | ch | tr | tr | 0 | tr | 0.3 | task | 0.077 0.002 0.419 | rest_go_safe |
| 895 | 1216.88 | collaborative | 1 | ch | tr | tr | 0 | tr | 0.3 | task | 0.077 0.002 0.419 | rest_go_safe |
| 896 | 1218.67 | collaborative | 1 | ch | tr | tr | 0 | tr | 0.3 | task | 0.077 0.002 0.419 | rest_go_safe |
| 897 | 1220.32 | collaborative | 1 | ch | tr | tr | 0 | tr | 0.3 | task | 0.077 0.002 0.419 | rest_go_safe |
| 898 | 1221.88 | collaborative | 1 | ch | tr | tr | 0 | tr | 0.3 | task | 0.077 0.002 0.419 | rest_go_safe |
| 899 | 1223.87 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 900 | 1263.49 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 901 | 1264.85 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 902 | 1266.29 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 903 | 1267.77 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 904 | 1269.11 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 905 | 1270.56 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 906 | 1271.88 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 907 | 1273.12 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 908 | 1274.39 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 909 | 1275.64 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 910 | 1277.17 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 911 | 1278.49 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 912 | 1279.73 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 913 | 1281.28 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 914 | 1282.78 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 915 | 1283.73 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 916 | 1284.99 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 917 | 1286.21 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 918 | 1287.30 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 919 | 1288.40 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 920 | 1289.49 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 921 | 1290.50 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 922 | 1291.80 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 923 | 1292.80 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 924 | 1293.99 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 925 | 1295.10 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 926 | 1296.59 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 927 | 1297.75 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 928 | 1298.78 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 929 | 1299.89 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 930 | 1300.91 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 931 | 1301.88 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 932 | 1302.83 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 933 | 1303.91 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 934 | 1305.40 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 935 | 1306.54 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 936 | 1307.76 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 937 | 1309.14 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 938 | 1310.42 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 939 | 1311.67 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 940 | 1312.85 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 941 | 1314.21 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 942 | 1315.44 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 943 | 1316.58 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 944 | 1317.80 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 945 | 1319.08 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 946 | 1320.64 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 947 | 1322.45 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 948 | 1323.76 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 949 | 1325.20 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 950 | 1326.47 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 951 | 1327.92 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 952 | 1328.87 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 953 | 1330.39 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 954 | 1331.48 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 955 | 1332.81 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 956 | 1333.99 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 957 | 1335.26 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 958 | 1336.62 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 959 | 1338.30 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 960 | 1339.69 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 961 | 1340.98 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 962 | 1342.34 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 963 | 1343.69 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 964 | 1344.92 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 965 | 1346.28 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 966 | 1347.64 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 967 | 1348.80 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 968 | 1350.18 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 969 | 1351.37 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 970 | 1352.69 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 971 | 1354.06 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 972 | 1355.47 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 973 | 1356.87 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 974 | 1358.12 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 975 | 1359.47 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 976 | 1360.87 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 977 | 1362.15 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 978 | 1363.45 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 979 | 1364.76 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 980 | 1366.18 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 981 | 1367.38 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 982 | 1368.66 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 983 | 1370.31 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 984 | 1371.79 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 985 | 1372.88 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 986 | 1373.95 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 987 | 1374.97 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 988 | 1376.11 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 989 | 1377.18 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 990 | 1378.35 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 991 | 1379.41 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 992 | 1380.31 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 993 | 1381.55 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 994 | 1382.57 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 995 | 1383.86 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 996 | 1385.15 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 997 | 1386.12 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 998 | 1387.10 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 999 | 1388.28 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1000 | 1389.31 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1001 | 1390.39 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1002 | 1391.53 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1003 | 1392.60 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1004 | 1393.82 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1005 | 1395.02 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1006 | 1396.33 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1007 | 1397.53 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1008 | 1398.60 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1009 | 1399.77 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1010 | 1400.87 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1011 | 1401.99 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1012 | 1402.96 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1013 | 1404.11 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1014 | 1405.31 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1015 | 1406.39 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1016 | 1407.44 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1017 | 1408.48 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1018 | 1409.60 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1019 | 1410.64 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1020 | 1411.71 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1021 | 1412.85 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1022 | 1413.89 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1023 | 1414.92 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1024 | 1416.06 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1025 | 1417.01 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1026 | 1418.11 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1027 | 1419.24 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1028 | 1420.31 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1029 | 1421.35 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1030 | 1422.37 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1031 | 1423.85 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1032 | 1425.41 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1033 | 1426.87 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1034 | 1427.84 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1035 | 1428.99 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1036 | 1430.02 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1037 | 1431.14 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1038 | 1432.17 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1039 | 1433.29 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1040 | 1434.23 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1041 | 1435.38 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1042 | 1436.36 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1043 | 1437.36 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1044 | 1438.47 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1045 | 1439.38 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1046 | 1440.43 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1047 | 1441.48 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1048 | 1442.47 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1049 | 1443.56 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1050 | 1444.63 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1051 | 1445.75 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1052 | 1446.73 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1053 | 1447.76 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1054 | 1448.89 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1055 | 1449.83 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1056 | 1451.02 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1057 | 1452.05 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1058 | 1453.13 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1059 | 1454.16 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1060 | 1455.27 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1061 | 1456.24 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1062 | 1457.36 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1063 | 1458.50 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1064 | 1459.53 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1065 | 1460.54 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1066 | 1461.51 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1067 | 1462.74 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1068 | 1464.22 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1069 | 1465.66 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1070 | 1466.89 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1071 | 1468.09 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1072 | 1469.04 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1073 | 1470.13 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1074 | 1471.06 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1075 | 1472.01 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1076 | 1473.17 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1077 | 1474.31 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1078 | 1475.28 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1079 | 1476.41 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1080 | 1477.41 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1081 | 1478.71 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1082 | 1479.67 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1083 | 1480.86 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1084 | 1481.95 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1085 | 1483.10 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1086 | 1484.13 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1087 | 1485.17 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1088 | 1486.07 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1089 | 1487.11 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1090 | 1488.16 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1091 | 1489.29 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1092 | 1490.35 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1093 | 1491.37 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1094 | 1492.45 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1095 | 1493.46 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1096 | 1494.53 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1097 | 1495.63 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1098 | 1496.76 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1099 | 1497.67 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1100 | 1498.79 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1101 | 1499.78 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1102 | 1500.72 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1103 | 1502.50 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1104 | 1503.82 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1105 | 1505.07 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1106 | 1506.34 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1107 | 1507.36 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1108 | 1508.39 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1109 | 1509.41 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1110 | 1510.44 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1111 | 1511.43 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1112 | 1512.53 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1113 | 1513.61 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1114 | 1514.62 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1115 | 1515.58 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1116 | 1516.68 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1117 | 1517.79 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1118 | 1519.56 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1119 | 1521.41 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1120 | 1522.72 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1121 | 1523.96 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1122 | 1525.19 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1123 | 1526.31 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1124 | 1527.59 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1125 | 1528.87 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1126 | 1530.25 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1127 | 1531.73 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1128 | 1533.04 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1129 | 1534.34 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1130 | 1535.52 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1131 | 1536.66 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1132 | 1537.72 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1133 | 1539.13 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1134 | 1540.68 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1135 | 1542.43 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1136 | 1543.49 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1137 | 1544.76 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1138 | 1546.10 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1139 | 1547.27 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1140 | 1548.40 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1141 | 1549.69 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1142 | 1551.07 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1143 | 1552.09 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1144 | 1553.11 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1145 | 1554.08 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1146 | 1555.40 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1147 | 1556.76 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1148 | 1558.04 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1149 | 1559.39 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1150 | 1560.61 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1151 | 1561.98 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1152 | 1563.23 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1153 | 1564.44 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1154 | 1565.71 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1155 | 1566.94 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1156 | 1568.03 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1157 | 1569.40 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1158 | 1570.91 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1159 | 1572.46 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1160 | 1573.64 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1161 | 1575.40 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1162 | 1577.30 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1163 | 1578.94 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1164 | 1580.43 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1165 | 1581.85 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1166 | 1582.98 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1167 | 1584.40 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1168 | 1585.65 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1169 | 1586.82 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1170 | 1588.88 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1171 | 1590.49 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1172 | 1591.73 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1173 | 1592.87 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1174 | 1594.11 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1175 | 1595.37 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1176 | 1596.55 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1177 | 1597.67 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1178 | 1598.82 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1179 | 1600.07 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1180 | 1601.35 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1181 | 1602.41 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1182 | 1603.70 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1183 | 1604.95 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1184 | 1606.06 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1185 | 1607.08 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1186 | 1608.22 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1187 | 1609.44 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1188 | 1610.57 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1189 | 1611.82 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1190 | 1612.85 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1191 | 1614.00 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1192 | 1615.37 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1193 | 1616.61 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1194 | 1617.84 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1195 | 1618.92 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1196 | 1623.38 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1197 | 1625.18 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1198 | 1626.77 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1199 | 1627.93 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1200 | 1629.30 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1201 | 1630.62 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1202 | 1631.72 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1203 | 1632.85 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1204 | 1634.09 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1205 | 1635.52 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1206 | 1637.18 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1207 | 1638.96 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1208 | 1640.52 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1209 | 1641.68 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1210 | 1643.07 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1211 | 1644.53 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1212 | 1645.75 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1213 | 1647.07 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1214 | 1648.38 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1215 | 1649.87 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1216 | 1651.13 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1217 | 1652.46 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1218 | 1654.19 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1219 | 1655.58 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1220 | 1656.89 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1221 | 1658.46 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1222 | 1659.71 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1223 | 1661.08 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1224 | 1662.28 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1225 | 1663.50 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1226 | 1664.65 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1227 | 1666.17 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1228 | 1667.47 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1229 | 1668.69 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1230 | 1669.83 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1231 | 1671.04 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1232 | 1672.21 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1233 | 1673.59 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1234 | 1674.90 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1235 | 1676.15 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1236 | 1677.41 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1237 | 1678.60 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1238 | 1679.82 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1239 | 1680.99 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1240 | 1682.15 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1241 | 1683.84 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1242 | 1686.55 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1243 | 1687.85 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1244 | 1688.87 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1245 | 1690.23 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1246 | 1691.34 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1247 | 1692.61 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1248 | 1693.81 | collaborative | 0 | tr | tr | ps | 10 | ps | 0.0 | session | - | session_stopped_go_safe |
| 1249 | 1695.39 | collaborative | 0 | tr | tr | ps | 10 | ps | 0.0 | session | - | session_stopped_go_safe |
| 1250 | 1696.74 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1251 | 1698.03 | collaborative | 0 | tr | tr | ps | 10 | ps | 0.0 | session | - | session_stopped_go_safe |
| 1252 | 1699.44 | collaborative | 0 | tr | tr | ps | 10 | ps | 0.0 | session | - | session_stopped_go_safe |
| 1253 | 1701.21 | collaborative | 0 | tr | tr | ps | 10 | ps | 0.0 | session | - | session_stopped_go_safe |
| 1254 | 1702.81 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1255 | 1704.21 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1256 | 1705.83 | collaborative | 0 | tr | tr | tr | 0 | tr | 0.3 | session | 0.077 0.002 0.419 | session_stopped_go_safe |
| 1257 | 1707.14 | collaborative | 0 | tr | tr | ps | 10 | ps | 0.0 | session | - | session_stopped_go_safe |
| 1258 | 1708.82 | collaborative | 0 | tr | tr | ps | 10 | ps | 0.0 | session | - | session_stopped_go_safe |
| 1259 | 1709.97 | collaborative | 0 | tr | tr | ps | 10 | ps | 0.0 | session | - | session_stopped_go_safe |
| 1260 | 1710.99 | collaborative | 0 | tr | tr | ps | 10 | ps | 0.0 | session | - | session_stopped_go_safe |
| 1261 | 1711.96 | collaborative | 0 | tr | tr | ps | 10 | ps | 0.0 | session | - | session_stopped_go_safe |
| 1262 | 1712.58 | collaborative | 0 | tr | tr | ps | 10 | ps | 0.0 | session | - | session_stopped_go_safe |
| 1263 | 1713.25 | collaborative | 0 | tr | tr | ps | 10 | ps | 0.0 | session | - | session_stopped_go_safe |
| 1264 | 1714.00 | collaborative | 0 | tr | tr | ps | 10 | ps | 0.0 | session | - | session_stopped_go_safe |
| 1265 | 1714.68 | collaborative | 0 | tr | tr | ps | 10 | ps | 0.0 | session | - | session_stopped_go_safe |
| 1266 | 1715.36 | collaborative | 0 | tr | tr | ps | 10 | ps | 0.0 | session | - | session_stopped_go_safe |
| 1267 | 1715.99 | collaborative | 0 | tr | tr | ps | 10 | ps | 0.0 | session | - | session_stopped_go_safe |
| 1268 | 1716.80 | collaborative | 0 | tr | tr | ps | 10 | ps | 0.0 | session | - | session_stopped_go_safe |
| 1269 | 1717.53 | collaborative | 0 | tr | tr | ps | 10 | ps | 0.0 | session | - | session_stopped_go_safe |
| 1270 | 1718.21 | collaborative | 0 | tr | tr | ps | 10 | ps | 0.0 | session | - | session_stopped_go_safe |
| 1271 | 1718.85 | collaborative | 0 | tr | tr | ps | 10 | ps | 0.0 | session | - | session_stopped_go_safe |
| 1272 | 1719.51 | collaborative | 0 | tr | tr | ps | 10 | ps | 0.0 | session | - | session_stopped_go_safe |
| 1273 | 1720.22 | collaborative | 0 | tr | tr | ps | 10 | ps | 0.0 | session | - | session_stopped_go_safe |
| 1274 | 1720.94 | collaborative | 0 | tr | tr | ps | 10 | ps | 0.0 | session | - | session_stopped_go_safe |
| 1275 | 1721.59 | collaborative | 0 | tr | tr | ps | 10 | ps | 0.0 | session | - | session_stopped_go_safe |
| 1276 | 1722.31 | collaborative | 0 | tr | tr | ps | 10 | ps | 0.0 | session | - | session_stopped_go_safe |
| 1277 | 1722.97 | collaborative | 0 | tr | tr | ps | 10 | ps | 0.0 | session | - | session_stopped_go_safe |
| 1278 | 1723.66 | collaborative | 0 | tr | tr | ps | 10 | ps | 0.0 | session | - | session_stopped_go_safe |
| 1279 | 1724.35 | collaborative | 0 | tr | tr | ps | 10 | ps | 0.0 | session | - | session_stopped_go_safe |
| 1280 | 1725.25 | collaborative | 0 | tr | tr | ps | 10 | ps | 0.0 | session | - | session_stopped_go_safe |
| 1281 | 1725.90 | collaborative | 0 | tr | tr | ps | 10 | ps | 0.0 | session | - | session_stopped_go_safe |
| 1282 | 1726.56 | collaborative | 0 | tr | tr | ps | 10 | ps | 0.0 | session | - | session_stopped_go_safe |
