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
    private double imuOffset = 0;

    public IMU(HardwareMap hardwareMap) {
        com.qualcomm.robotcore.hardware.IMU found = hardwareMap.tryGet(com.qualcomm.robotcore.hardware.IMU.class, "pinpoint");
        if (found == null) {
            found = hardwareMap.get(com.qualcomm.robotcore.hardware.IMU.class, "imu");
        }
        imuDevice = found;

        Parameters parameters = new Parameters(new IMUOrientationOnRobot(
                LogoFacingDirection.UP,
                UsbFacingDirection.FORWARD
        ));
        parameters.angleUnit = com.qualcomm.robotcore.hardware.IMU.AngleUnit.DEGREES;
        imuDevice.initialize(parameters);
    }

    public double getHeadingInDegrees() {
        Orientation angles = imuDevice.getRobotOrientation(
                AxesReference.INTRINSIC,
                IMUAxesOrder.ZYX,
                AngleUnit.DEGREES
        );
        return AngleUnit.normalizeDegrees(angles.firstAngle * -1 - imuOffset);
    }

    public double getHeadingInRadians() {
        Orientation angles = imuDevice.getRobotOrientation(
                AxesReference.INTRINSIC,
                IMUAxesOrder.ZYX,
                AngleUnit.RADIANS
        );
        return AngleUnit.normalizeRadians(angles.firstAngle);
    }

    public void setImuOffset(double offset) {
        imuOffset += offset;
    }

    public void resetIMU() {
        imuOffset = 0;
        imuDevice.resetYaw();
    }
}
