# AdvantageScope autonomous simulation

This branch runs the Rebuilt robot model with MapleSim and publishes WPILib struct telemetry that AdvantageScope can
render directly.

## Start a run

1. Initialize the branch-specific dependencies with `git submodule update --init --recursive`.
2. Start the robot simulation with `./gradlew simulateJava` (or the WPILib **Simulate Robot Code** command).
3. Open `elastic-layout.json` in Elastic and connect it to the local simulator.
4. On Elastic's **Autonomous** tab, select the alliance, strategy, starting position, and strategy-specific options.
   The choices are read again automatically when autonomous is enabled. **Preview Choices** is optional and shows the
   captured configuration on dashboard line 07.
5. In the Simulation Driver Station, select **Autonomous**, enable, and run the routine.

The simulation profile leaves the physical shooter, intake, and climber disabled. Custom autonomous routines treat
those mechanism actions as no-ops and continue through their drive states, allowing their paths to be reviewed safely.

## AdvantageScope field setup

Connect AdvantageScope live to NetworkTables at `localhost`, add a 2D or 3D field, and drag these topics onto it:

- `/MapleSim/Swerve/SimulationPose` — MapleSim ground-truth robot pose.
- `/MapleSim/Swerve/OdometryPose` — robot-code odometry pose for drift comparison.
- `/MapleSim/Auto/SelectedStartPose` — start pose selected in Elastic.
- `/MapleSim/Auto/ActivePath` — waypoints for the active Pure Pursuit segment.
- `/MapleSim/Auto/TargetPose` — endpoint of the active Pure Pursuit segment.
- `/MapleSim/Auto/ActualTrajectory` — sampled ground-truth trail for the current autonomous run.
- `/MapleSim/Field/Fuel` — simulated fuel positions as a `Pose3d[]`.

`/MapleSim/Auto/SelectedConfiguration` records the exact Elastic configuration used at enable, and
`/MapleSim/Auto/Active` indicates whether robot code is currently in autonomous mode. Swerve module states, chassis
speeds, heading, and 3D robot poses remain available under `/MapleSim/Swerve`.

## Dependency routing

The simulator needs small construction and telemetry hooks that are not available in upstream `trc492/frclib`.
Because simulation contributors cannot push that commit there, this branch points the `frclib` submodule at the
`sim` branch in `Wlute112/frclib`. The upstream `trc492/frclib` and `trc492/trclib` repositories remain untouched.
