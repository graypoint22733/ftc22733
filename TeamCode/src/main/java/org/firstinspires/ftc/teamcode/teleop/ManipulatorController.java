package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.robotcore.hardware.Gamepad;

import org.firstinspires.ftc.teamcode.subsystems.Indexer;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

public class ManipulatorController {
    public enum IndexerState { LEFT, RIGHT, HOLD }

    public record ManipulatorState(double yawPower, double flywheelPower, IndexerState indexerState) { }

    private final Turret turret;
    private final Indexer indexer;

    public ManipulatorController(Turret turret, Indexer indexer) {
        this.turret = turret;
        this.indexer = indexer;
    }

    public ManipulatorState updateFromGamepad(Gamepad gamepad) {
        double yawInput = gamepad.right_stick_x * SwerveTeleOpConfig.yawScale;
        turret.setYawPower(yawInput);

        double flywheelPower = gamepad.left_trigger - gamepad.right_trigger;
        if (Math.abs(flywheelPower) > 0.01) {
            turret.setFlywheelPower(flywheelPower);
        } else {
            turret.stopFlywheel();
        }

        IndexerState indexerState = IndexerState.HOLD;
        if (gamepad.dpad_left) {
            indexer.left();
            indexerState = IndexerState.LEFT;
        } else if (gamepad.dpad_right) {
            indexer.right();
            indexerState = IndexerState.RIGHT;
        } else if (gamepad.dpad_down) {
            indexer.neutral();
        }

        return new ManipulatorState(yawInput, flywheelPower, indexerState);
    }
}
