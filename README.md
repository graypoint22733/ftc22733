# Differential swerve starter

This repository has been trimmed down to the essentials needed to run a differential (diffy) swerve drivetrain on the FTC SDK. All other game-specific mechanisms and tools have been removed so you can focus on driving and tuning the base. The remaining hardware references assume a goBilda build: 520x drive motors, goBilda odometry pods, the goBilda Pinpoint IMU/odometry backbone, and Axon servos.

The project still retains the standard FTC scaffolding (Gradle settings, `FtcRobotController`, shared build scripts) so it builds in Android Studio like a normal multi-module SDK project; only the unused OpModes and utilities were removed.

## What remains
- **TeamCode module** with the classes required for diffy swerve and a few drivetrain-focused extras:
  - `SwerveDrive` and the supporting `IMU`, `maths` utilities, and `myDcMotorEx` helper (all assumed to be paired with goBilda 520x motors).
  - Two driving OpModes: `swerveTuner` (PID/offset tuning) and `diffyTurret` (adds turret/indexer controls and live odometry readout).
  - New subsystem stubs for a rotating flywheel turret and a left/right indexer servo (see `Turret.java` and `Indexer.java`).
  - Two-wheel odometry utilities (goBilda pods) for Road Runner pose tracking via `Encoder`, `LocalizationConstants`, and `TwoWheelTrackingLocalizer`.
- **FtcRobotController module** from the stock SDK so you can deploy to your Control Hub/Robot Controller.

## Hardware map expectations
The driving code expects the following hardware names in your configuration:

| Device | Configuration name |
| --- | --- |
| goBilda Pinpoint IMU (preferred) or stock BNO055 IMU | `pinpoint` (or fallback `imu`) |
| Module 1 drive motors | `mod1m1`, `mod1m2` |
| Module 2 drive motors | `mod2m1`, `mod2m2` |
| Module 3 drive motors | `mod3m1`, `mod3m2` |
| Module absolute encoders (analog) | `mod1E`, `mod2E`, `mod3E` |
| goBilda odometry pods (left/right) | `leftOdo`, `rightOdo` |
| goBilda Pinpoint module | `pinpoint` |
| Turret flywheel motor (goBilda 520x) | `flywheel` |
| Turret yaw motor (goBilda 520x) | `turretYaw` |
| Indexer Axon servo | `indexer` |

Adjust these names in the code if your wiring differs.

## Running the diffy
1. Open the project in Android Studio.
2. Select the `TeamCode` module and deploy to your Robot Controller.
3. On the Driver Station, choose an OpMode:
   - **swerveTuner** (TeleOp): drive with `gamepad1`; tune PID and module zeros live via the Dashboard config page.
   - **diffyTurret** (TeleOp): drive with `gamepad1`; run the flywheel/turret with `gamepad2` (triggers for flywheel, right stick X for yaw); shift the Axon indexer servo left/right with the `dpad` buttons (`Indexer` directly moves the servo positions). Telemetry defaults to the goBilda odometry pods with the Pinpoint IMU heading; Road Runner calculates pose via `TwoWheelTrackingLocalizer`.
4. Drive with `gamepad1`:
   - Left stick: translation
   - Right stick X: rotation (cubed for finer control)
5. Use the FTC Dashboard (optional) to tune the PID and angle offsets exposed as static fields in `swerveTuner`/`SwerveDrive`. The OpMode boots with the same PID defaults defined in `SwerveDrive` so the controllers are active immediately and can then be adjusted live.

That's it—no extra subsystems or autos to get in the way.
