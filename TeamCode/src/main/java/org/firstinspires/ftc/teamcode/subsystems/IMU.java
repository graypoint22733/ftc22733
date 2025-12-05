package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU.Parameters;
import com.qualcomm.robotcore.hardware.IMUAxesOrder;
import com.qualcomm.robotcore.hardware.IMUOrientationOnRobot;
import com.qualcomm.robotcore.hardware.IMUOrientationOnRobot.LogoFacingDirection;
import com.qualcomm.robotcore.hardware.IMUOrientationOnRobot.UsbFacingDirection;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * IMU wrapper that prefers the goBilda Pinpoint IMU if present (hardware name "pinpoint"),
 * but also works with the stock Control Hub BNO055 IMU (hardware name "imu").
 */
public class IMU {

    private final com.qualcomm.robotcore.hardware.IMU imuDevice;
    private final Parameters parameters;
    private double imuOffset = 0;
    private Orientation lastOrientation = new Orientation();

    public IMU(HardwareMap hardwareMap) {
        com.qualcomm.robotcore.hardware.IMU found = hardwareMap.tryGet(com.qualcomm.robotcore.hardware.IMU.class, "pinpoint");
        if (found == null) {
            found = hardwareMap.get(com.qualcomm.robotcore.hardware.IMU.class, "imu");
        }
        imuDevice = found;

        parameters = new Parameters(new IMUOrientationOnRobot(
                LogoFacingDirection.UP,
                UsbFacingDirection.FORWARD
        ));
        parameters.angleUnit = com.qualcomm.robotcore.hardware.IMU.AngleUnit.DEGREES;
        imuDevice.initialize(parameters);
    }

    public double getHeadingInDegrees() {
        Orientation angles = getOrientation(AngleUnit.DEGREES);
        return AngleUnit.normalizeDegrees(angles.firstAngle * -1 - imuOffset);
    }

    public double getHeadingInRadians() {
        Orientation angles = getOrientation(AngleUnit.RADIANS);
        double offsetRadians = Math.toRadians(imuOffset);
        return AngleUnit.normalizeRadians(angles.firstAngle * -1 - offsetRadians);
    }

    public void setImuOffset(double offset) {
        imuOffset += offset;
    }

    public void resetIMU() {
        imuOffset = 0;
        imuDevice.resetYaw();
    }

    /**
     * The Control Hub's BNO055 IMU occasionally reports a transient
     * "update error" after recent firmware changes. To keep the robot running
     * we recover by reinitializing the IMU once and returning the latest
     * successful orientation reading if a second attempt fails.
     */
    private Orientation getOrientation(AngleUnit angleUnit) {
        try {
            lastOrientation = imuDevice.getRobotOrientation(
                    AxesReference.INTRINSIC,
                    IMUAxesOrder.ZYX,
                    angleUnit
            );
            return lastOrientation;
        } catch (RuntimeException firstFailure) {
            if (isRecoverableBnoError(firstFailure)) {
                imuDevice.initialize(parameters);
                try {
                    lastOrientation = imuDevice.getRobotOrientation(
                            AxesReference.INTRINSIC,
                            IMUAxesOrder.ZYX,
                            angleUnit
                    );
                    return lastOrientation;
                } catch (RuntimeException secondFailure) {
                    return lastOrientation;
                }
            }
            throw firstFailure;
        }
    }

    private boolean isRecoverableBnoError(RuntimeException exception) {
        String message = exception.getMessage();
        return message != null && message.toLowerCase().contains("bno055") && message.toLowerCase().contains("update");
    }
}
