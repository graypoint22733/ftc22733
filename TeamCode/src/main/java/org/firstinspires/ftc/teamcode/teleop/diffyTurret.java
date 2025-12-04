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

@Config
@TeleOp(name = "diffyTurret", group = "Linear Opmode")
public class diffyTurret extends LinearOpMode {
    public static double yawScale = 0.5;

    @Override
    public void runOpMode() {
        SwerveDrive swerve = new SwerveDrive(telemetry, hardwareMap, true);
        Turret turret = new Turret(hardwareMap);
        Indexer indexer = new Indexer(hardwareMap);

        Encoder leftOdo = new Encoder(hardwareMap, "leftOdo");
        Encoder rightOdo = new Encoder(hardwareMap, "rightOdo");
        TwoWheelTrackingLocalizer localizer = new TwoWheelTrackingLocalizer(leftOdo, rightOdo, swerve::getHeading);

        waitForStart();
        leftOdo.reset();
        rightOdo.reset();

        while (opModeIsActive()) {
            // Drive with gamepad1
            swerve.drive(-gamepad1.left_stick_x, -gamepad1.left_stick_y,
                    gamepad1.right_stick_x * gamepad1.right_stick_x * gamepad1.right_stick_x);

            // IMU reset
            if (gamepad1.a) {
                swerve.resetIMU();
            }

            // Turret yaw + flywheel
            double yawInput = gamepad2.right_stick_x * yawScale;
            turret.setYawPower(yawInput);

            double flywheelPower = gamepad2.left_trigger - gamepad2.right_trigger;
            if (flywheelPower != 0) {
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

            localizer.update();
            Pose2d pose = localizer.getPoseEstimate();

            telemetry.addData("Pose", "x=%.2f, y=%.2f, h=%.1f", pose.getX(), pose.getY(), pose.getHeading());
            telemetry.addData("Flywheel", flywheelPower);
            telemetry.addData("Turret yaw", yawInput);
            telemetry.addData("Indexer", gamepad2.dpad_left ? "left" : gamepad2.dpad_right ? "right" : "hold");
            telemetry.update();
        }
    }
}
