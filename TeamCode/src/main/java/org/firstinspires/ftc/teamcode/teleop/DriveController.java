package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.SwerveDrive;

public class DriveController {
    public record DriveState(double translateX, double translateY, double rotation) { }

    private final SwerveDrive swerveDrive;

    public DriveController(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
    }

    public DriveState driveWithGamepad(Gamepad gamepad) {
        swerveDrive.setPIDCoeffs(SwerveTeleOpConfig.Kp, SwerveTeleOpConfig.Kd, SwerveTeleOpConfig.Ki,
                SwerveTeleOpConfig.Kf, SwerveTeleOpConfig.Kl);
        swerveDrive.setModuleAdjustments(SwerveTeleOpConfig.module1Adjust, SwerveTeleOpConfig.module2Adjust,
                SwerveTeleOpConfig.module3Adjust);

        double translateX = -gamepad.left_stick_x * SwerveTeleOpConfig.driveScale;
        double translateY = -gamepad.left_stick_y * SwerveTeleOpConfig.driveScale;
        double rotationInput = gamepad.right_stick_x;
        double rotation = Math.pow(rotationInput, 3) * SwerveTeleOpConfig.rotationScale;

        swerveDrive.drive(translateX, translateY, rotation);

        if (gamepad.a) {
            swerveDrive.resetIMU();
        }

        return new DriveState(translateX, translateY, rotation);
    }
}
