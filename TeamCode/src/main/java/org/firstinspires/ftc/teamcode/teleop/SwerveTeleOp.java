package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utility.Encoder;
import org.firstinspires.ftc.teamcode.utility.TwoWheelTrackingLocalizer;

/**
 * Consolidated TeleOp for the competition robot. Handles driving, turret control,
 * and indexer management from the two gamepads while still exposing PID tuning
 * hooks through the dashboard.
 */
@Config
@TeleOp(name = "SwerveTeleOp", group = "Linear Opmode")
public class SwerveTeleOp extends LinearOpMode {
    // Drive tuning exposed to the dashboard
    public static double Kp = 0.1, Kd = 0.002, Ki = 2, Kf = 0.5, Kl = 0.5;
    public static double module1Adjust = -10, module2Adjust = -10, module3Adjust = -45;
    public static double driveScale = 1.0, rotationScale = 1.0;

    // Turret tuning
    public static double yawScale = 0.5;

    @Override
    public void runOpMode() {
        // Subsystems
        SwerveDrive swerve = new SwerveDrive(telemetry, hardwareMap, true);
        Turret turret = new Turret(hardwareMap);
        Indexer indexer = new Indexer(hardwareMap);

        // Localization helpers
        Encoder leftOdo = new Encoder(hardwareMap, "leftOdo");
        Encoder rightOdo = new Encoder(hardwareMap, "rightOdo");
        TwoWheelTrackingLocalizer localizer = new TwoWheelTrackingLocalizer(leftOdo, rightOdo, swerve::getHeading);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();
        leftOdo.reset();
        rightOdo.reset();

        while (opModeIsActive()) {
            // Keep PID/module configuration in sync with dashboard values
            swerve.setPIDCoeffs(Kp, Kd, Ki, Kf, Kl);
            swerve.setModuleAdjustments(module1Adjust, module2Adjust, module3Adjust);

            // Drive with gamepad1: translation on left stick, rotation on right stick
            double translateX = -gamepad1.left_stick_x * driveScale;
            double translateY = -gamepad1.left_stick_y * driveScale;
            double rotationInput = gamepad1.right_stick_x;
            double rotation = Math.pow(rotationInput, 3) * rotationScale; // cube for finer control near zero

            swerve.drive(translateX, translateY, rotation);

            // IMU reset for field-centric driving
            if (gamepad1.a) {
                swerve.resetIMU();
            }

            // Turret yaw and flywheel on gamepad2
            double yawInput = gamepad2.right_stick_x * yawScale;
            turret.setYawPower(yawInput);

            double flywheelPower = gamepad2.left_trigger - gamepad2.right_trigger;
            if (Math.abs(flywheelPower) > 0.01) {
                turret.setFlywheelPower(flywheelPower);
            } else {
                turret.stopFlywheel();
            }

            // Indexer control from dpad
            if (gamepad2.dpad_left) {
                indexer.left();
            } else if (gamepad2.dpad_right) {
                indexer.right();
            } else if (gamepad2.dpad_down) {
                indexer.neutral();
            }

            // Update localization and telemetry
            localizer.update();
            Pose2d pose = localizer.getPoseEstimate();

            telemetry.addData("Pose", "x=%.2f, y=%.2f, h=%.1f", pose.getX(), pose.getY(), pose.getHeading());
            telemetry.addData("Drive", "x=%.2f y=%.2f rot=%.2f", translateX, translateY, rotation);
            telemetry.addData("Flywheel", flywheelPower);
            telemetry.addData("Turret yaw", yawInput);
            telemetry.addData("Indexer", gamepad2.dpad_left ? "left" : gamepad2.dpad_right ? "right" : "hold");
            telemetry.update();
        }
    }
}
