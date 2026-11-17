# Navigation implementation

Status: Docker images build, standalone C++ regression tests pass, and ROS 2
packages compile. The supplied simulator has completed goals (6, -1) and (-6, 2),
including a return route around the central cylinder. See verified results below.

## Run from Windows PowerShell

Start Docker Desktop in Linux container mode. ROS 2 Humble is supplied by the
project's base image; a separate Windows ROS installation is unnecessary.

```powershell
cd path\to\wato_asd_training
powershell -ExecutionPolicy Bypass -File .\run.ps1 Build
powershell -ExecutionPolicy Bypass -File .\run.ps1 Up
```

The first build downloads images and simulator dependencies and may take a while.
The robot image runs the standalone C++ tests before compiling the ROS packages.
If the build fails, retain the complete first error and its surrounding lines.

Open https://app.foxglove.dev and make a Foxglove WebSocket connection to
`ws://localhost:8765`. Import `config/wato_asd_training_foxglove_config .json`
(the space before `.json` is present in the original filename). Use `sim_world`
as the fixed frame. Display `/lidar`, `/costmap`, `/map`, `/path`, and
`/odom/filtered`. Give the simulator a few seconds to settle before setting a goal.

```powershell
powershell -ExecutionPolicy Bypass -File .\run.ps1 Goal -X -6 -Y 2
powershell -ExecutionPolicy Bypass -File .\run.ps1 Logs
```

The goal coordinates refer to the wheel axle in `sim_world`. The example is a
candidate first test, not a verified successful route. You can also publish a
`geometry_msgs/msg/PointStamped` to `/goal_point` from Foxglove in `sim_world`.
Avoid teleoperation while the autonomous controller is running: both would
publish to `/cmd_vel`.

Stop the containers when finished:

```powershell
powershell -ExecutionPolicy Bypass -File .\run.ps1 Down
```

## ROS nodes

| Node | Inputs | Output | Behavior |
|---|---|---|---|
| Costmap | `/lidar` LaserScan | `/costmap` OccupancyGrid | Ray-clears observed free cells, marks endpoints, and inflates obstacles. |
| Map memory | `/costmap`, `/odom/filtered`, TF | `/map` OccupancyGrid | Transforms scans at their observation timestamp and fuses known cells at 4 Hz, including when stationary. |
| Planner | `/map`, `/odom/filtered`, `/goal_point` | `/path` Path | Eight-neighbor weighted A*, no diagonal corner cutting, route validation at 2 Hz; replanning when required, goal completion and timeouts. |
| Control | `/path`, `/odom/filtered`, `/lidar` | `/cmd_vel` Twist | Pure pursuit at 20 Hz, rotate-first behavior, speed limits, stale-input stop and lidar collision guard. |

Algorithm code is in each package's `*_core.cpp` and `*_core.hpp`; subscriptions,
timers, publications, and ROS conversions are in `*_node.cpp` and `*_node.hpp`.
`navigation_common` supplies shared geometry and grid types. Core algorithms can
be tested without installing ROS, using `bash tests/run_core_tests.sh` on Linux
with a C++17 compiler.

## Geometry and map decisions

- The simulator's global frame is `sim_world`, not `map`.
- The supplied odometry tracks the lidar. In the supplied SDF, the lidar sits
  0.8 m ahead of the chassis frame and the wheel axle is 0.5 m behind it. The
  planner and controller therefore subtract a 1.3 m forward offset to recover
  the differential-drive axle position. This matters for pure pursuit kinematics.
- The 2 m by 1 m chassis extends 1.5 m forward of the axle. A conservative
  circular radius of 1.85 m includes the chassis, wheels, and clearance. The
  local costmap adds a cell-diagonal quantization margin; tune only after viewing
  the actual footprint and clearances in simulation.
- Local grid: 42 m square at 0.1 m/cell. Global grid: 60 m square at 0.2 m/cell,
  centered at the world origin. The supplied arena occupies approximately
  -15 to +15 m in each direction.
- Unknown cells are -1. They remain traversable with a higher cost to allow
  navigation before the whole map is observed. Measured obstacles have cost 100, footprint inflation has cost 99, and both
  are blocked for normal planning. Costs 1-98 discourage skimming obstacles.
  If the current cell is inflation-only, a recovery may relax inflation within
  0.6 m of the start, provided clearance to nearby observed obstacles does not
  fall below its starting value. Physical obstacles and blocked goals are never
  cleared. The independent lidar collision guard still applies.
- Fusion max-reduces all local cells that land in one global cell before applying
  that observation. New unknown data never erases previous observations.
- The map publishes on startup. The planner republishes or replaces its path
  every 0.5 s. Empty paths stop motion. A goal expires after 180 s or 30 s without
  position progress; publish another goal to retry.

## Tests and remaining verification

`tests/core_tests.cpp` covers scan endpoints/free/unknown space, invalid readings,
inflation, rotated map fusion, unknown memory retention, free updates, wall detours,
unreachable goals, bounds, corner cutting, controller signs/limits/stopping, and
closed-loop convergence around turns. The tests passed during the user-run Docker build. XML manifests and Python
launch syntax were also checked.

Before submitting:

1. Rebuild after code changes and ensure the core tests and ROS compilation pass.
2. Verify scan-frame TF lookup works, `/map` updates, and the robot follows a
   goal without collisions. Test several goals around the central cylinder and
   boxes, an occupied goal, a new goal during motion, and stale input stopping.
3. Tune clearance/lookahead/speed if the conservative defaults prevent progress.
4. Record real simulator navigation in Foxglove or Gazebo. Capture the live run; successful terminal traces do not replace a demonstration video.
5. Publish the code to your GitHub repository and submit its link plus the video
   in the assignment-completions channel. Credit collaborators and AI assistance
   accurately. No code or message has been submitted on your behalf.

This is a static-environment assignment implementation, not a complete navigation
stack: no localization estimation, moving-obstacle prediction, recovery maneuvers,
or dynamic map expansion are included. Lidar collision checks use a conservative
circle and a short commanded-motion horizon; they are not a hardware safety system.

## Replanning and recovery

Measured obstacle cells (100) are separate from footprint inflation (99).
Bounded outward start recovery handles discretization at inflation boundaries;
regression tests ensure measured obstacles and inflated goals remain blocked.
The planner keeps a usable route. When it must replan, proximity to its previous
route adds a nonnegative cost to reduce unnecessary switching between obstacle
corridors. Every candidate still obeys collision constraints. A new goal clears
that preference.

## Verified runtime results: route-preference-3

The user-run build completed successfully with the added regression tests.
Runtime traces confirm:

- Goal (6, -1), starting near (3.665, 3.930): reached in approximately 16 seconds
  after goal publication (18 seconds into the trace). Final axle
  (5.8149, -0.8855), error 0.218 m; zero command after completion.
- Return goal (-6, 2), starting near (5.815, -0.886): the trace shows navigation
  around the north side of the central cylinder. The planner logged Goal reached
  about 49.3 seconds after publication. Final axle (-5.7782, 1.9391), error
  0.230 m; zero command and empty path after completion.

These are successful navigation/stop tests on the supplied simulation. No video
has been inspected here, and these logs alone do not establish collision-free
behavior for all routes. Submit a recording with the repository link as required by the assignment.
The earlier (5, -1) test endpoint became inflated cost 99 as observations updated;
its failure is retained as a blocked-goal test, not a successful navigation run.
