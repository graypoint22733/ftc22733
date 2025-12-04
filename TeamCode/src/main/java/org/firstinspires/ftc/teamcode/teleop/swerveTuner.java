package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;

@Config
@TeleOp(name="swerveTuner", group="Linear Opmode")
public class swerveTuner extends LinearOpMode {

    // Tune these from the FTC Dashboard config page while the OpMode is running.
    // Defaults mirror the base values configured inside SwerveDrive so we don't
    // accidentally zero-out the drive controllers when the OpMode starts.
    public static double Kp = 0.1, Kd = 0.002, Ki = 2, Kf = 0.5, Kl = 0.5;
    public static double module1Adjust = -10, module2Adjust = -10, module3Adjust = -45;

    public void runOpMode() {
        telemetry.addData("Status", "Initialized");

        // Differential swerve drive
        SwerveDrive swerve = new SwerveDrive(telemetry, hardwareMap, true);

        waitForStart();
        while (opModeIsActive()) {
            // Push live config changes into the drive before applying stick inputs
            swerve.setPIDCoeffs(Kp, Kd, Ki, Kf, Kl);
            swerve.setModuleAdjustments(module1Adjust, module2Adjust, module3Adjust);

            // Translation: left stick; Rotation: right stick X (cubed for finer control)
            swerve.drive(-gamepad1.left_stick_x, -gamepad1.left_stick_y,
                    gamepad1.right_stick_x * gamepad1.right_stick_x * gamepad1.right_stick_x);

            if (gamepad1.a) {
                swerve.resetIMU();
            }

            telemetry.update();
        }
    }
}
