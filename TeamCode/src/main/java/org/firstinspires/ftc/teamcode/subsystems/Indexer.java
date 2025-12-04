package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Placeholder indexer that rotates left/right based on driver buttons.
 */
public class Indexer {
    private final Servo servo;

    private static final double LEFT_POSITION = 0.2;
    private static final double NEUTRAL_POSITION = 0.5;
    private static final double RIGHT_POSITION = 0.8;

    public Indexer(HardwareMap hardwareMap) {
        this.servo = hardwareMap.get(Servo.class, "indexer");
        this.servo.setPosition(NEUTRAL_POSITION);
    }

    public void left() {
        servo.setPosition(LEFT_POSITION);
    }

    public void right() {
        servo.setPosition(RIGHT_POSITION);
    }

    public void neutral() {
        servo.setPosition(NEUTRAL_POSITION);
    }
}
