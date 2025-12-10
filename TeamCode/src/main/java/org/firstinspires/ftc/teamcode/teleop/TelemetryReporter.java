package org.firstinspires.ftc.teamcode.teleop;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class TelemetryReporter {
    private final Telemetry telemetry;
    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    public TelemetryReporter(Telemetry telemetry) {
        this.telemetry = telemetry;
    }

    public void addInitializationStatus() {
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    public void reportLoop(Gamepad gamepad1, Pose2d pose, DriveController.DriveState driveState,
                            ManipulatorController.ManipulatorState manipulatorState) {
        TelemetryPacket packet = new TelemetryPacket();

        telemetry.addData("Pose", "x=%.2f, y=%.2f, h=%.1f", pose.getX(), pose.getY(), pose.getHeading());
        telemetry.addData("Drive", "x=%.2f y=%.2f rot=%.2f", driveState.getTranslateX(), driveState.getTranslateY(),
                driveState.getRotation());
        telemetry.addData("Flywheel", manipulatorState.getFlywheelPower());
        telemetry.addData("Turret yaw", manipulatorState.getYawPower());
        telemetry.addData("Indexer", manipulatorState.getIndexerState());
        telemetry.addData("IMU Reset", gamepad1.a);
        telemetry.update();

        packet.put("poseX", pose.getX());
        packet.put("poseY", pose.getY());
        packet.put("heading", pose.getHeading());
        packet.put("driveX", driveState.getTranslateX());
        packet.put("driveY", driveState.getTranslateY());
        packet.put("rotation", driveState.getRotation());
        packet.put("flywheelPower", manipulatorState.getFlywheelPower());
        packet.put("yawPower", manipulatorState.getYawPower());
        packet.put("indexer", manipulatorState.getIndexerState().name());
        packet.put("imuReset", gamepad1.a);
        packet.put("leftTrigger", gamepad1.left_trigger);
        packet.put("rightTrigger", gamepad1.right_trigger);
        packet.put("speedScalar", Range.clip(SwerveTeleOpConfig.driveScale, 0, 1));

        dashboard.sendTelemetryPacket(packet);

    }
}
